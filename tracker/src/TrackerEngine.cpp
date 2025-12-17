// Amoeba fiducial tracker.
//
// First, try to detect real Reactable "amoeba" fiducials using the
// original libfidtrack segmentation and tree-based decoder from the
// local reacTIVision sources. When no valid fiducials are found in a
// frame, fall back to a simple blob-based tracker that assigns stable
// numeric IDs based on spatial proximity, so tests and synthetic
// scenarios keep working even without printed markers.
#include "TrackerEngine.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace rectai::tracker {

namespace {

constexpr int kMaxFiducials = 32;
constexpr int kMinRecognisedMarkerId = 0;
constexpr int kMaxRecognisedMarkerId = 215;

// Upper bound on the resolution used by the amoeba segmenter.
// The camera may run at a higher resolution, but we downscale
// frames before feeding them into libfidtrack. This keeps the
// amount of work in segment.c roughly proportional to this
// fixed area instead of the native sensor resolution.
constexpr int kMaxProcessingWidth = 1280;
constexpr int kMaxProcessingHeight = 720;

// How many consecutive "only Otsu" or "only adaptive" successes
// are required before flipping the preference between Otsu-first
// and adaptive-first.
constexpr int kSuccessStreakForPreferenceFlip = 10;

// When preferring Otsu first, we only pay the cost of adaptive
// thresholding if Otsu has failed for this many consecutive frames.
constexpr int kOtsuFailuresBeforeTryingAdaptive = 2;

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

    // Clamp the internal processing resolution to a reasonable
    // upper bound. This reduces the amount of work done by the
    // segmenter (build_regions / merge_regions, etc.) when the
    // camera runs at HD or higher resolutions.
    if (frameWidth_ > kMaxProcessingWidth || frameHeight_ > kMaxProcessingHeight) {
        const float scaleX = static_cast<float>(kMaxProcessingWidth) / static_cast<float>(frameWidth_);
        const float scaleY = static_cast<float>(kMaxProcessingHeight) / static_cast<float>(frameHeight_);
        const float scale = std::min(scaleX, scaleY);

        frameWidth_ = std::max(1, static_cast<int>(std::round(frameWidth_ * scale)));
        frameHeight_ = std::max(1, static_cast<int>(std::round(frameHeight_ * scale)));
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

    // Helper lambdas for the four concrete thresholding strategies
    // we may use, in different orders depending on recent history.
    auto runOtsu = [&](int flags) -> TrackedObjectList {
        cv::threshold(resized, binary, 0.0, 255.0, flags);
        return detectWithBinary(binary);
    };

    auto runAdaptive = [&](int flags) -> TrackedObjectList {
        cv::adaptiveThreshold(resized, binary, 255.0, cv::ADAPTIVE_THRESH_MEAN_C,
                              flags, 21, 5.0);
        return detectWithBinary(binary);
    };

    // Decide which family of thresholds to try first based on
    // recent detection history: if we tend to detect only with
    // adaptive, try those first; otherwise, prefer Otsu.
    auto tryOtsuThenAdaptive = [&]() -> TrackedObjectList {
        TrackedObjectList fiducials = runOtsu(cv::THRESH_BINARY | cv::THRESH_OTSU);
        if (!fiducials.empty()) {
            consecutiveOtsuOnlySuccess_++;
            consecutiveAdaptiveOnlySuccess_ = 0;
            consecutiveOtsuFailures_ = 0;
            framesSinceAnyDetection_ = 0;
            return fiducials;
        }

        ++consecutiveOtsuFailures_;

        // Do not pay the cost of adaptive thresholding for isolated
        // Otsu failures; require a short streak of failures first.
        if (consecutiveOtsuFailures_ < kOtsuFailuresBeforeTryingAdaptive) {
            ++framesSinceAnyDetection_;
            consecutiveOtsuOnlySuccess_ = 0;
            consecutiveAdaptiveOnlySuccess_ = 0;
            return {};
        }

        // Try inverted Otsu once we are already in a failure streak.
        fiducials = runOtsu(cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        if (!fiducials.empty()) {
            consecutiveOtsuOnlySuccess_++;
            consecutiveAdaptiveOnlySuccess_ = 0;
            consecutiveOtsuFailures_ = 0;
            framesSinceAnyDetection_ = 0;
            return fiducials;
        }

        // Finally, pay the more expensive adaptive thresholding
        // passes when global thresholds are consistently failing.
        fiducials = runAdaptive(cv::THRESH_BINARY);
        if (!fiducials.empty()) {
            consecutiveAdaptiveOnlySuccess_++;
            consecutiveOtsuOnlySuccess_ = 0;
            consecutiveOtsuFailures_ = 0;
            framesSinceAnyDetection_ = 0;
            return fiducials;
        }

        fiducials = runAdaptive(cv::THRESH_BINARY_INV);
        if (!fiducials.empty()) {
            consecutiveAdaptiveOnlySuccess_++;
            consecutiveOtsuOnlySuccess_ = 0;
            consecutiveOtsuFailures_ = 0;
            framesSinceAnyDetection_ = 0;
            return fiducials;
        }

        ++framesSinceAnyDetection_;
        consecutiveOtsuOnlySuccess_ = 0;
        consecutiveAdaptiveOnlySuccess_ = 0;
        return {};
    };

    auto tryAdaptiveThenOtsu = [&]() -> TrackedObjectList {
        TrackedObjectList fiducials = runAdaptive(cv::THRESH_BINARY);
        if (!fiducials.empty()) {
            consecutiveAdaptiveOnlySuccess_++;
            consecutiveOtsuOnlySuccess_ = 0;
            consecutiveOtsuFailures_ = 0;
            framesSinceAnyDetection_ = 0;
            return fiducials;
        }

        fiducials = runAdaptive(cv::THRESH_BINARY_INV);
        if (!fiducials.empty()) {
            consecutiveAdaptiveOnlySuccess_++;
            consecutiveOtsuOnlySuccess_ = 0;
            consecutiveOtsuFailures_ = 0;
            framesSinceAnyDetection_ = 0;
            return fiducials;
        }

        // If adaptive fails, fall back to the cheaper Otsu
        // thresholds before giving up.
        fiducials = runOtsu(cv::THRESH_BINARY | cv::THRESH_OTSU);
        if (!fiducials.empty()) {
            consecutiveOtsuOnlySuccess_++;
            consecutiveAdaptiveOnlySuccess_ = 0;
            consecutiveOtsuFailures_ = 0;
            framesSinceAnyDetection_ = 0;
            return fiducials;
        }

        fiducials = runOtsu(cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        if (!fiducials.empty()) {
            consecutiveOtsuOnlySuccess_++;
            consecutiveAdaptiveOnlySuccess_ = 0;
            consecutiveOtsuFailures_ = 0;
            framesSinceAnyDetection_ = 0;
            return fiducials;
        }

        ++framesSinceAnyDetection_;
        consecutiveOtsuOnlySuccess_ = 0;
        consecutiveAdaptiveOnlySuccess_ = 0;
        return {};
    };

    TrackedObjectList fiducials;

    // Flip preference between Otsu-first and adaptive-first based
    // on which family has been more successful recently.
    if (!preferAdaptiveFirst_ && consecutiveAdaptiveOnlySuccess_ >= kSuccessStreakForPreferenceFlip) {
        preferAdaptiveFirst_ = true;
    } else if (preferAdaptiveFirst_ && consecutiveOtsuOnlySuccess_ >= kSuccessStreakForPreferenceFlip) {
        preferAdaptiveFirst_ = false;
    }

    if (preferAdaptiveFirst_) {
        fiducials = tryAdaptiveThenOtsu();
    } else {
        fiducials = tryOtsuThenAdaptive();
    }

    return fiducials;
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
