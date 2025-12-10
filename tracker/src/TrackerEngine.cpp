// Amoeba fiducial tracker.
//
// First, try to detect real Reactable "amoeba" fiducials using the
// original libfidtrack segmentation and tree-based decoder from the
// local reacTIVision sources. When no valid fiducials are found in a
// frame, fall back to a simple blob-based tracker that assigns stable
// numeric IDs based on spatial proximity, so tests and synthetic
// scenarios keep working even without printed markers.
#include "TrackerEngine.h"

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace rectai::tracker {

namespace {

constexpr int kMaxFiducials = 1024;
constexpr int kMinRecognisedMarkerId = 1;
constexpr int kMaxRecognisedMarkerId = 107;
constexpr int kMinFiducialImageSize = 32;
constexpr int kFiducialCropSize = 64;
constexpr float kMinBlackPixelRatio = 0.10F;

} // namespace

TrackerEngine::TrackerEngine() = default;

TrackerEngine::~TrackerEngine()
{
    if (amoebaInitialised_) {
        terminate_segmenter(&segmenter_);
        terminate_fidtrackerX(&fidtrackerX_);
        terminate_treeidmap(&treeIdMap_);
        amoebaInitialised_ = false;
    }
}

bool TrackerEngine::initialise(const int /*cameraIndex*/, const int requestedWidth, const int requestedHeight, std::string& /*errorMessage*/)
{
    frameWidth_ = requestedWidth;
    frameHeight_ = requestedHeight;
    lastObjects_.clear();
    nextId_ = 1;
    if (frameWidth_ <= 0 || frameHeight_ <= 0) {
        initialised_ = false;
        return false;
    }

    // Initialise amoeba fiducial tracking using the default tree
    // set bundled with reacTIVision ("default" amoeba markers).
    initialize_treeidmap(&treeIdMap_, "default");
    initialize_segmenter(&segmenter_, frameWidth_, frameHeight_, treeIdMap_.max_adjacencies);
    initialize_fidtrackerX(&fidtrackerX_, &treeIdMap_, nullptr);

    amoebaInitialised_ = true;
    initialised_ = true;
    return initialised_;
}

bool TrackerEngine::isInitialised() const noexcept
{
    return initialised_;
}

TrackedObjectList TrackerEngine::processFrame(const cv::Mat& frame)
{
    return processFrameInternal(frame, nullptr);
}

TrackedObjectList TrackerEngine::processFrame(const cv::Mat& frame, cv::Mat& debugFrame)
{
    return processFrameInternal(frame, &debugFrame);
}

TrackedObjectList TrackerEngine::processFrameInternal(const cv::Mat& frame, cv::Mat* debugFrame)
{
    TrackedObjectList result;

    if (!initialised_) {
        return result;
    }

    if (frame.empty() || frame.cols <= 0 || frame.rows <= 0) {
        return result;
    }

    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }

    cv::Mat resized;
    if (gray.cols != frameWidth_ || gray.rows != frameHeight_) {
        cv::resize(gray, resized, cv::Size(frameWidth_, frameHeight_));
    } else {
        resized = gray;
    }

    const float width = static_cast<float>(frameWidth_ > 0 ? frameWidth_ : resized.cols);
    const float height = static_cast<float>(frameHeight_ > 0 ? frameHeight_ : resized.rows);

    cv::Mat blurred;
    cv::GaussianBlur(resized, blurred, cv::Size(5, 5), 0.0);

    cv::Mat binary;
    cv::adaptiveThreshold(blurred, binary, 255.0, cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY, 21, 5.0);

    if (debugFrame != nullptr) {
        binary.copyTo(*debugFrame);
    }

    // 1) Try to detect real amoeba fiducials using libfidtrack on a
    // binarized image, as expected by libfidtrack's segmenter
    // (pixels must be 0 or 255).
    if (amoebaInitialised_) {
        TrackedObjectList fiducials = detectAmoebaFiducials(binary);
        if (!fiducials.empty()) {
            // When we have encoded IDs from the pattern itself there is no
            // need to keep proximity-based state.
            lastObjects_.clear();
            nextId_ = 1;
            return fiducials;
        }
    }

    // 2) Fallback: blob-based tracker with nearest-neighbour ID assignment.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        lastObjects_.clear();
        return result;
    }

    TrackedObjectList detections;
    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < 500.0) {
            continue;
        }

        const cv::Moments mu = cv::moments(contour);
        if (std::fabs(mu.m00) < 1e-5) {
            continue;
        }

        const float cx = static_cast<float>(mu.m10 / mu.m00);
        const float cy = static_cast<float>(mu.m01 / mu.m00);

        float angleRad = 0.0F;
        if (contour.size() >= 5U) {
            const cv::RotatedRect box = cv::fitEllipse(contour);
            angleRad = static_cast<float>(box.angle * (M_PI / 180.0));
        }

        TrackedObject obj;
        obj.id = -1;
        obj.x_norm = cx / width;
        obj.y_norm = cy / height;
        obj.angle_rad = angleRad;
        obj.alive = true;
        detections.push_back(obj);
    }

    if (detections.empty()) {
        lastObjects_.clear();
        return result;
    }

    std::vector<bool> prevUsed(lastObjects_.size(), false);
    constexpr float kMaxDistSq = 0.05F * 0.05F;

    for (auto& det : detections) {
        int bestIndex = -1;
        float bestDistSq = kMaxDistSq;

        for (std::size_t i = 0; i < lastObjects_.size(); ++i) {
            if (prevUsed[i]) {
                continue;
            }

            const auto& prev = lastObjects_[i];
            const float dx = det.x_norm - prev.x_norm;
            const float dy = det.y_norm - prev.y_norm;
            const float distSq = dx * dx + dy * dy;
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestIndex = static_cast<int>(i);
            }
        }

        if (bestIndex >= 0) {
            det.id = lastObjects_[static_cast<std::size_t>(bestIndex)].id;
            prevUsed[static_cast<std::size_t>(bestIndex)] = true;
        } else {
            det.id = nextId_++;
        }
    }

    lastObjects_ = detections;
    result = detections;
    return result;
}

TrackedObjectList TrackerEngine::detectAmoebaFiducials(const cv::Mat& binaryFrame)
{
    TrackedObjectList result;

    if (binaryFrame.empty() || binaryFrame.cols != frameWidth_ || binaryFrame.rows != frameHeight_) {
        return result;
    }

    if (binaryFrame.type() != CV_8UC1) {
        return result;
    }

    if (binaryFrame.cols < kMinFiducialImageSize || binaryFrame.rows < kMinFiducialImageSize) {
        return result;
    }

    FiducialX fiducials[kMaxFiducials];

    step_segmenter(&segmenter_, binaryFrame.ptr<unsigned char>());

#ifndef NDEBUG
    sanity_check_region_initial_values(&segmenter_);
#endif

    const int count = find_fiducialsX(fiducials, kMaxFiducials, &fidtrackerX_, &segmenter_, frameWidth_, frameHeight_);

    for (int i = 0; i < count; ++i) {
        const int fidId = fiducials[i].id;
        // Only keep fiducials within the allowed ID range
        if (fidId <= kMinRecognisedMarkerId || fidId >= kMaxRecognisedMarkerId) {
            continue;
        }

        const float xNorm = fiducials[i].x / static_cast<float>(frameWidth_);
        const float yNorm = fiducials[i].y / static_cast<float>(frameHeight_);

        if (xNorm < 0.0F || xNorm > 1.0F || yNorm < 0.0F || yNorm > 1.0F) {
            continue;
        }

        // Additional validation: crop a region around the fiducial centre
        // on the binarized image and ensure there is enough black content.
        const int halfSize = kFiducialCropSize / 2;
        int cx = static_cast<int>(fiducials[i].x);
        int cy = static_cast<int>(fiducials[i].y);

        if (cx < 0) {
            cx = 0;
        }
        if (cy < 0) {
            cy = 0;
        }
        if (cx >= frameWidth_) {
            cx = frameWidth_ - 1;
        }
        if (cy >= frameHeight_) {
            cy = frameHeight_ - 1;
        }

        int x0 = cx - halfSize;
        int y0 = cy - halfSize;
        if (x0 < 0) {
            x0 = 0;
        }
        if (y0 < 0) {
            y0 = 0;
        }

        int x1 = x0 + kFiducialCropSize;
        int y1 = y0 + kFiducialCropSize;
        if (x1 > frameWidth_) {
            x1 = frameWidth_;
        }
        if (y1 > frameHeight_) {
            y1 = frameHeight_;
        }

        const int roiWidth = x1 - x0;
        const int roiHeight = y1 - y0;
        if (roiWidth <= 0 || roiHeight <= 0) {
            continue;
        }

        const cv::Rect roi(x0, y0, roiWidth, roiHeight);
        const cv::Mat patch = binaryFrame(roi);

        const int totalPixels = patch.rows * patch.cols;
        if (totalPixels <= 0) {
            continue;
        }

        const int whitePixels = cv::countNonZero(patch);
        const int blackPixels = totalPixels - whitePixels;
        const float blackRatio = static_cast<float>(blackPixels) / static_cast<float>(totalPixels);

        if (blackRatio < kMinBlackPixelRatio) {
            // Not enough black content around the detected fiducial, likely noise.
            continue;
        }

        TrackedObject obj;
        obj.id = fidId;
        obj.x_norm = xNorm;
        obj.y_norm = yNorm;
        obj.angle_rad = fiducials[i].angle;
        obj.alive = true;
        result.push_back(obj);
    }

    return result;
}

} // namespace rectai::tracker
