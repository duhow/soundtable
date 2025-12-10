// rectai-tracker entry point.
// Supports two modes:
//   --mode=synthetic (default): send a single synthetic object.
//   --mode=live:      delegate per-frame processing to TrackerEngine.

#include <cmath>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "OscSender.h"
#include "TrackerEngine.h"
#include "TrackerState.h"

namespace {

enum class RunMode {
	Synthetic,
	Live
};

RunMode parseMode(int argc, char** argv)
{
	for (int i = 1; i < argc; ++i) {
		const std::string arg{argv[i]};
		if (arg == "--mode=live") {
			return RunMode::Live;
		}
		if (arg == "--mode=synthetic") {
			return RunMode::Synthetic;
		}
	}

	return RunMode::Synthetic;
}

std::string mapLogicalId(int markerId)
{
	switch (markerId) {
	case 1: return "osc1";
	case 2: return "filter1";
	default: return "osc1";
	}
}

} // namespace

int main(int argc, char** argv)
{
	const auto mode = parseMode(argc, argv);

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

	rectai::tracker::TrackerEngine trackerEngine;
	rectai::tracker::TrackerState trackerState;
	std::string initError;

	if (mode == RunMode::Live) {
		const int width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
		const int height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
		if (!trackerEngine.initialise(0, width, height, initError)) {
			std::cerr << "[rectai-tracker] Failed to initialise TrackerEngine: "
					  << initError << std::endl;
			return 1;
		}
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

		if (mode == RunMode::Synthetic) {
			if (oscSender.isOk()) {
				phase += 0.01F;
				if (phase > 2.0F * static_cast<float>(M_PI)) {
					phase -= 2.0F * static_cast<float>(M_PI);
				}

				const float x = 0.5F + 0.3F * std::sin(phase);
				const float y = 0.5F;
				const float angle = 0.0F;

				(void)oscSender.sendObject(1, "osc1", x, y, angle);
			}
		} else {
			const auto objects = trackerEngine.processFrame(frame);

			trackerState.update(objects);

			if (oscSender.isOk()) {
				for (const auto& obj : objects) {
					(void)oscSender.sendObject(obj.id, mapLogicalId(obj.id),
											   obj.x_norm, obj.y_norm, obj.angle_rad);
				}

				const auto removedIds = trackerState.collectRemovals(15);
				for (const int id : removedIds) {
					(void)oscSender.sendRemove(id);
				}
			}
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
