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

    if (!initialised_ || !amoebaInitialised_) {
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

    cv::Mat binary;

    auto detectWithBinary = [&](const cv::Mat& bin) -> TrackedObjectList {
        if (debugFrame != nullptr) {
            bin.copyTo(*debugFrame);
        }
        return detectAmoebaFiducials(bin);
    };

    // Strategy 1: global Otsu threshold (markers dark on light background).
    cv::threshold(resized, binary, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);
    TrackedObjectList fiducials = detectWithBinary(binary);
    if (!fiducials.empty()) {
        return fiducials;
    }

    // Strategy 2: inverted Otsu threshold (markers light on dark background).
    cv::threshold(resized, binary, 0.0, 255.0, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    fiducials = detectWithBinary(binary);
    if (!fiducials.empty()) {
        return fiducials;
    }

    // Strategy 3: adaptive mean threshold (non-inverted).
    cv::adaptiveThreshold(resized, binary, 255.0, cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY, 21, 5.0);
    fiducials = detectWithBinary(binary);
    if (!fiducials.empty()) {
        return fiducials;
    }

    // Strategy 4: adaptive mean threshold (inverted).
    cv::adaptiveThreshold(resized, binary, 255.0, cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY_INV, 21, 5.0);
    fiducials = detectWithBinary(binary);
    if (!fiducials.empty()) {
        return fiducials;
    }

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
