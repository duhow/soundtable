 #pragma once

#include <array>
#include <cstdint>
#include <set>
#include <string>

#include <opencv2/core.hpp>

#include "TrackerTypes.h"

extern "C" {
#include "segment.h"
#include "fidtrackX.h"
#include "treeidmap.h"
}

namespace soundtable::tracker {

class TrackerEngine {
public:
    TrackerEngine();

    ~TrackerEngine();

    bool initialise(int cameraIndex,
                    int requestedWidth,
                    int requestedHeight,
                    std::string& errorMessage,
                    bool enableDownscale = true);

    [[nodiscard]] bool isInitialised() const noexcept;

    [[nodiscard]] TrackedObjectList processFrame(const cv::Mat& frame);

    // Optional debug variant: fills debugFrame with the processed image
    // used internally for fiducial detection (e.g. binarized frame).
    [[nodiscard]] TrackedObjectList processFrame(const cv::Mat& frame, cv::Mat& debugFrame);

    // When enabled, always run a single threshold filter instead of
    // evaluating all available filters and performing adaptive
    // per-fiducial training.
    void setOnlySingleFilter(bool enabled) noexcept { onlySingleFilter_ = enabled; }

    // Convenience helper used by the command-line front-end to select
    // a specific threshold filter by index (0..3) and enable single
    // filter mode in one call.
    void setOnlySingleFilterByIndex(bool enabled, int filterIndex) noexcept;

private:
    enum class ThresholdFilter : std::uint8_t {
        OtsuBinary = 0,
        OtsuBinaryInv,
        AdaptiveBinary,
        AdaptiveBinaryInv
    };

    static constexpr int kNumThresholdFilters = 4;

    bool initialised_{};
    int frameWidth_{};
    int frameHeight_{};
    TrackedObjectList lastObjects_;

    bool amoebaInitialised_{};
    Segmenter segmenter_{};
    TreeIdMap treeIdMap_{};
    FidtrackerX fidtrackerX_{};

    // If true, bypass the adaptive multi-filter selection logic and
    // always run a single threshold filter per frame.
    bool onlySingleFilter_{false};

    // State for per-fiducial filter evaluation:
    // - when a fiducial is first seen, we start a 30-frame window
    //   where all filters are evaluated in parallel and we count
    //   how many times each filter manages to see that fiducial.
    // - the filter with the highest hit count becomes the "active"
    //   filter until the fiducial disappears for several frames.
    int primaryFiducialId_{-1};
    int trainingFramesRemaining_{};
    bool hasActiveFilter_{};
    ThresholdFilter activeFilter_{ThresholdFilter::OtsuBinary};
    bool primaryLostLastFrame_{};
    int framesSincePrimarySeen_{};
    std::array<int, kNumThresholdFilters> filterSuccessCount_{};
    std::array<std::set<int>, kNumThresholdFilters> filterSeenIds_{};

    [[nodiscard]] TrackedObjectList detectAmoebaFiducials(const cv::Mat& binaryFrame);

    [[nodiscard]] TrackedObjectList processFrameInternal(const cv::Mat& frame, cv::Mat* debugFrame);

    [[nodiscard]] static int filterIndex(ThresholdFilter filter) noexcept;
};

} // namespace soundtable::tracker
