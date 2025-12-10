#pragma once

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include "TrackerTypes.h"

namespace rectai::tracker {

class TrackerEngine {
public:
    TrackerEngine();

    bool initialise(int cameraIndex, int requestedWidth, int requestedHeight, std::string& errorMessage);

    [[nodiscard]] bool isInitialised() const noexcept;

    [[nodiscard]] TrackedObjectList processFrame(const cv::Mat& frame) const;

private:
    bool initialised_{};
    int frameWidth_{};
    int frameHeight_{};
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParameters_;
};

} // namespace rectai::tracker
