#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "TrackerTypes.h"

extern "C" {
#include "segment.h"
#include "fidtrackX.h"
#include "treeidmap.h"
}

namespace rectai::tracker {

class TrackerEngine {
public:
    TrackerEngine();

    ~TrackerEngine();

    bool initialise(int cameraIndex, int requestedWidth, int requestedHeight, std::string& errorMessage);

    [[nodiscard]] bool isInitialised() const noexcept;

    [[nodiscard]] TrackedObjectList processFrame(const cv::Mat& frame);

    // Optional debug variant: fills debugFrame with the processed image
    // used internally for fiducial detection (e.g. binarized frame).
    [[nodiscard]] TrackedObjectList processFrame(const cv::Mat& frame, cv::Mat& debugFrame);

private:
    bool initialised_{};
    int frameWidth_{};
    int frameHeight_{};
    TrackedObjectList lastObjects_;

    bool amoebaInitialised_{};
    Segmenter segmenter_{};
    TreeIdMap treeIdMap_{};
    FidtrackerX fidtrackerX_{};

    // Simple adaptive strategy state for choosing between
    // global Otsu thresholds and adaptive thresholds based on
    // recent detection history.
    int framesSinceAnyDetection_{};
    int consecutiveOtsuOnlySuccess_{};
    int consecutiveAdaptiveOnlySuccess_{};
    int consecutiveOtsuFailures_{};
    bool preferAdaptiveFirst_{};

    [[nodiscard]] TrackedObjectList detectAmoebaFiducials(const cv::Mat& binaryFrame);

    [[nodiscard]] TrackedObjectList processFrameInternal(const cv::Mat& frame, cv::Mat* debugFrame);
};

} // namespace rectai::tracker
