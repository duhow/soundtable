#include "TrackerEngine.h"

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace rectai::tracker {

TrackerEngine::TrackerEngine() = default;

bool TrackerEngine::initialise(const int /*cameraIndex*/, const int requestedWidth, const int requestedHeight, std::string& /*errorMessage*/)
{
    frameWidth_ = requestedWidth;
    frameHeight_ = requestedHeight;

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detectorParameters_ = cv::aruco::DetectorParameters::create();

    initialised_ = static_cast<bool>(dictionary_) && static_cast<bool>(detectorParameters_);
    return initialised_;
}

bool TrackerEngine::isInitialised() const noexcept
{
    return initialised_;
}

TrackedObjectList TrackerEngine::processFrame(const cv::Mat& frame) const
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

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;

    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detectorParameters_);

    if (ids.empty()) {
        return result;
    }

    const float width = static_cast<float>(frameWidth_ > 0 ? frameWidth_ : frame.cols);
    const float height = static_cast<float>(frameHeight_ > 0 ? frameHeight_ : frame.rows);

    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (corners[i].size() != 4U) {
            continue;
        }

        float sumX = 0.0F;
        float sumY = 0.0F;
        for (const auto& p : corners[i]) {
            sumX += p.x;
            sumY += p.y;
        }

        const float centerX = sumX / 4.0F;
        const float centerY = sumY / 4.0F;

        const cv::Point2f p0 = corners[i][0];
        const cv::Point2f p1 = corners[i][1];
        const float dx = p1.x - p0.x;
        const float dy = p1.y - p0.y;
        const float angle = std::atan2(dy, dx);

        TrackedObject obj;
        obj.id = ids[i];
        obj.x_norm = centerX / width;
        obj.y_norm = centerY / height;
        obj.angle_rad = angle;
        obj.alive = true;

        result.push_back(obj);
    }

    return result;
}

} // namespace rectai::tracker
