#include <iostream>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "OscSender.h"

// Minimal tracking service prototype.
// For now it:
//   - Opens the default camera.
//   - Prints approximate FPS.
//   - Sends synthetic OSC messages to the JUCE core so that
//     the full tracking → core → UI path can be exercised.

int main()
{
    cv::VideoCapture capture(0);
    if (!capture.isOpened()) {
        std::cerr << "[rectai-tracker] Failed to open default camera." << std::endl;
        return 1;
    }

    std::cout << "[rectai-tracker] Camera opened. Press Ctrl+C to exit." << std::endl;

    // OSC sender towards the JUCE core (localhost:3333 by default).
    OscSender oscSender{"127.0.0.1", 3333};
    if (!oscSender.isOk()) {
        std::cerr << "[rectai-tracker] OSC sender is not available."
                  << " Core will not receive tracking updates." << std::endl;
    }

    cv::Mat frame;
    auto start_time = static_cast<double>(cv::getTickCount());
    int frames = 0;

    // Simple phase accumulator to move a synthetic object horizontally.
    float phase = 0.0F;

    while (true) {
        if (!capture.read(frame) || frame.empty()) {
            std::cerr << "[rectai-tracker] Error capturing frame." << std::endl;
            break;
        }

        ++frames;

        // Send a synthetic object update on every frame so that the
        // JUCE core can show some movement even without real markers.
        if (oscSender.isOk()) {
            // Advance phase slowly; wrap it to avoid overflow.
            phase += 0.01F;
            if (phase > 2.0F * static_cast<float>(M_PI)) {
                phase -= 2.0F * static_cast<float>(M_PI);
            }

            const float x = 0.5F + 0.3F * std::sin(phase);  // [0.2, 0.8]
            const float y = 0.5F;
            const float angle = 0.0F;

            // Tracking id 1, logical id "osc1" to match the example scene.
            (void)oscSender.sendObject(1, "osc1", x, y, angle);
        }
        const double now = static_cast<double>(cv::getTickCount());
        const double elapsed_sec = (now - start_time) / cv::getTickFrequency();
        if (elapsed_sec >= 1.0) {
            const double fps = static_cast<double>(frames) / elapsed_sec;
            std::cout << "[rectai-tracker] Approx FPS: " << fps << '\n';

            // Reset counters so we can keep reporting periodically.
            start_time = now;
            frames = 0;
        }
    }

    return 0;
}
