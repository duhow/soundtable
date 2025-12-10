#include <cassert>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "TrackerEngine.h"

using rectai::tracker::TrackerEngine;
using rectai::tracker::TrackedObjectList;

namespace {

cv::Mat loadFiducialImage(const std::string& filename)
{
    std::vector<std::string> candidates{
        filename,
        "tests/" + filename,
        "../tests/" + filename,
        "../../tests/" + filename,
    };

#ifdef RECTAI_SOURCE_DIR
    candidates.push_back(std::string(RECTAI_SOURCE_DIR) + "/tests/" + filename);
#endif

    for (const auto& path : candidates) {
        cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (!img.empty()) {
            return img;
        }
    }

    std::cerr << "rectai-tracker-tests: could not load fiducial image '"
              << filename << "' from any known path" << std::endl;
    assert(false && "Failed to load fiducial PNG for tracker test");
    return {};
}

int expectedIdFromFilename(const std::string& filename)
{
    const auto underscorePos = filename.find_last_of('_');
    const auto dotPos = filename.find_last_of('.');
    assert(underscorePos != std::string::npos);
    assert(dotPos != std::string::npos);
    assert(dotPos > underscorePos);

    const std::string idStr = filename.substr(underscorePos + 1, dotPos - underscorePos - 1);
    return std::stoi(idStr);
}

} // namespace

int main()
{
    // Load the first image to initialise the engine dimensions
    const std::string firstFile = "fiducial_30.jpg";
    const cv::Mat firstImage = loadFiducialImage(firstFile);
    assert(!firstImage.empty());

    std::string error;
    TrackerEngine engine;
    assert(engine.initialise(0, firstImage.cols, firstImage.rows, error));

    const std::vector<std::string> files{"fiducial_30.jpg", "fiducial_55.jpg"};

    for (const auto& file : files) {
        const cv::Mat img = (file == firstFile) ? firstImage : loadFiducialImage(file);
        const TrackedObjectList objects = engine.processFrame(img);

        assert(!objects.empty());

        const int expectedId = expectedIdFromFilename(file);
        std::cout << "rectai-tracker-tests: file " << file << " (expected ID "
                  << expectedId << ") -> IDs:";

        bool foundExpectedId = false;
        for (const auto& obj : objects) {
            std::cout << " " << obj.id;
            if (obj.id == expectedId) {
                foundExpectedId = true;
            }
        }
        std::cout << std::endl;

        assert(foundExpectedId && "Expected amoeba ID matching filename in TrackerEngine output");
    }

    std::cout << "rectai-tracker-tests: OK" << std::endl;
    return 0;
}
