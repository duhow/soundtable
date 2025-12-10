// rectai-tracker entry point.
// Supports two modes:
//   --mode=synthetic (default): send a single synthetic object.
//   --mode=live:      delegate per-frame processing to TrackerEngine.

#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

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

	return RunMode::Live;
}

bool hasDebugViewFlag(int argc, char** argv)
{
	for (int i = 1; i < argc; ++i) {
		const std::string arg{argv[i]};
		if (arg == "--debug-view") {
			return true;
		}
	}
	return false;
}

// Maps physical tracker marker IDs to logical module IDs used by the
// JUCE core. Logical IDs must match the module ids created when loading
// the default Reactable patch (see com.reactable/Resources/default.rtp
// and ReactableRtpLoader).
//
// Current mapping:
//   56 -> Volume      (module id "1")
//   44 -> Sequencer   (module id "5")
//   45 -> Sequencer   (module id "6")
//   51 -> Tempo       (module id "2")
//   53 -> Tonalizer   (module id "0")
//   29 -> Filter      (module id "8")
//   30 -> Delay       (module id "10")
//   31 -> Delay       (module id "11")
//
// Unknown markers fall back to the Output module ("-1") so they are
// visible but do not control any specific instrument module.
std::string mapLogicalId(int markerId)
{
	switch (markerId) {
	case 56: return "1";   // Volume
	case 44: return "5";   // Sequencer A
	case 45: return "6";   // Sequencer B
	case 51: return "2";   // Tempo
	case 53: return "0";   // Tonalizer
	case 29: return "8";   // Filter
	case 30: return "10";  // Delay A
	case 31: return "11";  // Delay B
	default: return "-1";  // Output / unknown
	}
}

} // namespace

int main(int argc, char** argv)
{
	const auto mode = parseMode(argc, argv);
	const bool debugView = hasDebugViewFlag(argc, argv);

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
	rectai::tracker::TrackedObjectList lastStableObjectsForDebug;
	std::string initError;

	// Require several consecutive detections of the same fiducial ID,
	// with only small spatial jitter between frames, before considering
	// it a valid object. This helps to filter out spurious and unstable
	// detections coming from the camera feed.
	std::unordered_map<int, int> consecutiveDetectionsById;
	std::unordered_map<int, std::pair<float, float>> lastPositionById;

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
	cv::Mat debugFrame;
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
			rectai::tracker::TrackedObjectList objects;
			if (debugView) {
				objects = trackerEngine.processFrame(frame, debugFrame);
			} else {
				objects = trackerEngine.processFrame(frame);
			}

			// Update per-ID consecutive detection counters for this frame.
			// We only increase the streak if the fiducial stays close to its
			// previous position; large jumps reset the counter.
			constexpr float kMaxJitterNorm = 0.1F; // ~10% of table width/height.
			const float kMaxJitterSq = kMaxJitterNorm * kMaxJitterNorm;
			std::unordered_set<int> visibleIds;
			for (const auto& obj : objects) {
				visibleIds.insert(obj.id);
				auto countIt = consecutiveDetectionsById.find(obj.id);
				auto posIt = lastPositionById.find(obj.id);
				if (countIt == consecutiveDetectionsById.end() || posIt == lastPositionById.end()) {
					consecutiveDetectionsById[obj.id] = 1;
					lastPositionById[obj.id] = {obj.x_norm, obj.y_norm};
				} else {
					const float dx = obj.x_norm - posIt->second.first;
					const float dy = obj.y_norm - posIt->second.second;
					const float distSq = dx * dx + dy * dy;
					if (distSq > kMaxJitterSq) {
						// Too much movement between frames: start a new streak.
						countIt->second = 1;
					} else {
						++countIt->second;
					}
					posIt->second = {obj.x_norm, obj.y_norm};
				}
			}
			// Reset counters and last known positions for IDs not visible
			// in this frame so that we only accept detections seen in
			// consecutive frames.
			for (auto it = consecutiveDetectionsById.begin(); it != consecutiveDetectionsById.end();) {
				if (visibleIds.find(it->first) == visibleIds.end()) {
					lastPositionById.erase(it->first);
					it = consecutiveDetectionsById.erase(it);
				} else {
					++it;
				}
			}

			// Build the list of "stable" objects that have been seen in at
			// least three consecutive frames. Only these will be reflected in
			// lifetime tracking and OSC messages.
			rectai::tracker::TrackedObjectList stableObjects;
			stableObjects.reserve(objects.size());
			for (const auto& obj : objects) {
				const auto it = consecutiveDetectionsById.find(obj.id);
				const int consecutiveFrames = (it != consecutiveDetectionsById.end()) ? it->second : 0;
				if (consecutiveFrames >= 0) { // NOTE: increase value if you need more stability or accuracy
					stableObjects.push_back(obj);
				}
			}

			// Update lifetime tracking only with stable objects so that
			// removals are also consistent with the stability threshold.
			trackerState.update(stableObjects);
			lastStableObjectsForDebug = stableObjects;

			if (oscSender.isOk()) {
				for (const auto& obj : stableObjects) {
					const std::string logicalId = mapLogicalId(obj.id);
					std::cout << "[rectai-tracker] fiducial " << obj.id
							<< " -> logicalId=" << logicalId
							<< " x=" << obj.x_norm
							<< " y=" << obj.y_norm << '\n';
					(void)oscSender.sendObject(obj.id, logicalId,
									 obj.x_norm, obj.y_norm, obj.angle_rad);
				}


				// Any id that has been missing for several frames is treated as
				// removed from the musical area, similar to a TUIO remove event.
				const auto removedIds = trackerState.collectRemovals(15);
				for (const int id : removedIds) {
					std::cout << "[rectai-tracker] fiducial " << id
						      << " removed from tracking" << '\n';
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

		if (debugView) {
			// Show the same kind of image that TrackerEngine consumes
			// (single-channel grayscale/binary) so the debug view matches
			// the fiducial tracking pipeline instead of the raw colour frame.
			cv::Mat sourceForDebug;
			if (mode == RunMode::Live && !debugFrame.empty()) {
				// In live mode, use the processed binary/grayscale frame
				// produced inside TrackerEngine::processFrame.
				sourceForDebug = debugFrame;
			} else {
				// In synthetic mode (or if for some reason we don't have
				// a debug frame), convert the camera frame to grayscale so
				// the visualisation is still comparable to reacTIVision.
				if (frame.channels() == 3) {
					cv::cvtColor(frame, sourceForDebug, cv::COLOR_BGR2GRAY);
				} else {
					sourceForDebug = frame;
				}
			}

			cv::Mat display;
			cv::flip(sourceForDebug, display, 1);

			cv::Mat displayColor;
			if (display.channels() == 1) {
				cv::cvtColor(display, displayColor, cv::COLOR_GRAY2BGR);
			} else {
				displayColor = display;
			}

			if (mode == RunMode::Live) {
				const int imgWidth = displayColor.cols;
				const int imgHeight = displayColor.rows;
				const int halfSize = 20;

				for (const auto& obj : lastStableObjectsForDebug) {
					int cx = static_cast<int>(obj.x_norm * static_cast<float>(imgWidth));
					int cy = static_cast<int>(obj.y_norm * static_cast<float>(imgHeight));
					if (cx < 0) cx = 0;
					if (cy < 0) cy = 0;
					if (cx >= imgWidth) cx = imgWidth - 1;
					if (cy >= imgHeight) cy = imgHeight - 1;

					const int cxFlipped = imgWidth - 1 - cx;
					const cv::Point topLeft(cxFlipped - halfSize, cy - halfSize);
					const cv::Point bottomRight(cxFlipped + halfSize, cy + halfSize);
					cv::rectangle(displayColor, topLeft, bottomRight, cv::Scalar(0, 0, 255), 2);
				}
			}

			cv::imshow("rectai-tracker debug", displayColor);
			const int key = cv::waitKey(1);
			if (key == 27 || key == 'q' || key == 'Q') {
				std::cout << "[rectai-tracker] Debug view exit requested by user" << '\n';
				break;
			}
		}
	}

	if (debugView) {
		cv::destroyWindow("rectai-tracker debug");
	}

	return 0;
}
