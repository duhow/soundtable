// rectai-tracker entry point.
// Supports two modes:
//   --mode=synthetic (default): send a single synthetic object.
//   --mode=live:      delegate per-frame processing to TrackerEngine.

#include <cmath>
#include <cctype>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <argparse/argparse.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include "OscSender.h"
#include "TrackerEngine.h"
#include "TrackerState.h"
#include "TuioNegotiation.h"

namespace {

enum class RunMode {
	Synthetic,
	Live
};

struct Resolution {
	int width{};
	int height{};
};

enum class PixelFormat {
	Any,
	Mjpg,
	Yuyv
};

std::optional<Resolution> parseResolutionString(const std::string& value)
{
	const auto xPos = value.find('x');
	if (xPos == std::string::npos || xPos == 0 || xPos == value.size() - 1) {
		return std::nullopt;
	}

	const std::string widthStr = value.substr(0, xPos);
	const std::string heightStr = value.substr(xPos + 1);

	try {
		const int width = std::stoi(widthStr);
		const int height = std::stoi(heightStr);
		if (width <= 0 || height <= 0) {
			return std::nullopt;
		}
		return Resolution{width, height};
	} catch (const std::exception&) {
		return std::nullopt;
	}
}

std::optional<PixelFormat> parseCameraFormatString(const std::string& input)
{
	std::string value = input;
	for (char& ch : value) {
		ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
	}

	if (value == "mjpg" || value == "mjpeg") {
		return PixelFormat::Mjpg;
	}
	if (value == "yuyv" || value == "yuy2") {
		return PixelFormat::Yuyv;
	}
	if (value.empty()) {
		return std::nullopt;
	}

	std::cerr << "[rectai-tracker] Invalid --camera-format '" << value
	          << "'. Supported values: mjpg, yuyv." << std::endl;
	return std::nullopt;
}

std::optional<int> parseFilterIndexString(const std::string& input)
{
	if (input.empty()) {
		return std::nullopt;
	}

	std::string value = input;
	for (char& ch : value) {
		ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
	}

	if (value == "otsu" || value == "otsu-binary" || value == "otsu_binary") {
		return 0;
	}
	if (value == "otsu-inv" || value == "otsu_inverse" || value == "otsu-binary-inv") {
		return 1;
	}
	if (value == "adaptive" || value == "adaptive-binary" || value == "adaptive_binary") {
		return 2;
	}
	if (value == "adaptive-inv" || value == "adaptive_inverse" || value == "adaptive-binary-inv") {
		return 3;
	}

	std::cerr << "[rectai-tracker] Invalid --filter value '" << value
	          << "'. Supported values: otsu, otsu-inv, adaptive, adaptive-inv." << std::endl;
	return std::nullopt;
}

RunMode parseRunModeString(const std::string& mode)
{
	if (mode == "synthetic") {
		return RunMode::Synthetic;
	}
	return RunMode::Live;
}

// Maps physical tracker marker IDs to logical module IDs used by the
// JUCE core. Logical IDs must match the module ids created when loading
// the default Reactable patch (see com.reactable/Resources/default.rtp
// and ReactableRtpLoader).
//
// Unknown markers fall back to the Output module ("-1") so they are
// visible but do not control any specific instrument module.
std::string mapLogicalId(int markerId)
{
	switch (markerId) {
	case 51: return "2";   // Tempo
	case 53: return "0";   // Tonalizer
	case 54: return "2";   // Tempo
	case 56: return "1";   // Volume
	case 44: return "5";   // Sequencer A
	case 45: return "6";   // Sequencer B
	case 24: return "24";  // Sample A
	case 46: return "24";  // Sample A
	case 47: return "29";  // Sample B
	case 29: return "8";   // Filter
	case 30: return "10";  // Delay A
	case 31: return "11";  // Delay B
	case 48: return "12";  // Modulator (Flanger)
	case 26: return "46";  // Oscillator A
	case 27: return "47";  // Oscillator B
	default: return "-1";  // Output / unknown
	}
}

} // namespace

int main(int argc, char** argv)
{
	argparse::ArgumentParser program("rectai-tracker", "0.1.0", argparse::default_arguments::all, true);
	program.add_description("rectai-tracker: camera fiducial tracker for RectaiTable.");
	program.add_argument("-m", "--mode")
	    .help("Select run mode (live tracking or synthetic object).")
	    .default_value(std::string{"live"})
	    .choices("live", "synthetic");
	program.add_argument("-d", "--camera")
	    .help("Select camera index (default: 0). Run v4l2-ctl --list-devices on Linux to see available cameras.")
	    .scan<'i', int>()
	    .default_value(0);
	program.add_argument("-q", "--camera-format")
	    .help("Request camera pixel format (Linux V4L2 backends). Supported: mjpg, yuyv.")
	    .default_value(std::string{});
	program.add_argument("-s", "--resolution")
	    .help("Request capture resolution, e.g. 1280x720.")
	    .default_value(std::string{});
	program.add_argument("-f", "--fps")
	    .help("Request capture framerate in FPS.")
	    .scan<'i', int>()
	    .default_value(0);
	program.add_argument("-af", "--no-autofocus")
	    .help("Disable continuous camera autofocus when supported.")
	    .flag();
	program.add_argument("-w", "--viewer")
	    .help("Show debug thresholded view from TrackerEngine.")
	    .flag();
	program.add_argument("-ds", "--no-downscale")
	    .help("Disable internal downscaling in TrackerEngine.")
	    .flag();
	program.add_argument("--osc")
	    .help("Force legacy OSC output instead of TUIO 1.1.")
	    .flag();
	program.add_argument("--only-filter")
	    .help("Always run a single threshold filter (no adaptive multi-filter training).")
	    .flag();
	program.add_argument("-l", "--filter")
	    .help("Select threshold filter (implies --only-filter). Supported: otsu, otsu-inv, adaptive, adaptive-inv.")
	    .default_value(std::string{});

	try {
		program.parse_args(argc, argv);
	} catch (const std::exception& err) {
		std::cerr << err.what() << std::endl;
		std::cerr << program;
		return 1;
	}

	const auto mode = parseRunModeString(program.get<std::string>("--mode"));
	const bool debugView = program.get<bool>("--viewer");
	const bool noDownscale = program.get<bool>("--no-downscale");
	const bool disableAutofocus = program.get<bool>("--no-autofocus");
	const bool forceOsc = program.get<bool>("--osc");
	const bool onlyFilter = program.get<bool>("--only-filter");
	const int cameraIndex = program.get<int>("--camera");

	std::optional<int> requestedFps;
	{
		const int fpsValue = program.get<int>("--fps");
		if (fpsValue > 0) {
			requestedFps = fpsValue;
		} else if (fpsValue < 0) {
			std::cerr << "[rectai-tracker] Invalid --fps '" << fpsValue
			          << "' (must be > 0). Using camera default fps." << std::endl;
		}
	}

	std::optional<Resolution> requestedResolution;
	if (program.is_used("--resolution")) {
		const auto resString = program.get<std::string>("--resolution");
		requestedResolution = parseResolutionString(resString);
	}

	std::optional<PixelFormat> requestedCameraFormat;
	if (program.is_used("--camera-format")) {
		requestedCameraFormat = parseCameraFormatString(program.get<std::string>("--camera-format"));
	}

	std::optional<int> filterIndex;
	if (program.is_used("--filter")) {
		filterIndex = parseFilterIndexString(program.get<std::string>("--filter"));
	}

	cv::VideoCapture capture;
#if defined(__linux__)
	capture.open(cameraIndex, cv::CAP_V4L2);
	if (!capture.isOpened()) {
		capture.open(cameraIndex);
	}
#else
	capture.open(cameraIndex);
#endif
	if (!capture.isOpened()) {
		std::cerr << "[rectai-tracker] Failed to open camera index " << cameraIndex << "." << std::endl;
		return 1;
	}

	if (requestedCameraFormat.has_value()) {
		int fourcc = 0;
		switch (*requestedCameraFormat) {
		case PixelFormat::Mjpg:
			fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
			break;
		case PixelFormat::Yuyv:
			fourcc = cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V');
			break;
		case PixelFormat::Any:
		default:
			break;
		}

		if (fourcc != 0) {
			(void)capture.set(cv::CAP_PROP_FOURCC, fourcc);
		}
	}

	if (disableAutofocus) {
		if (!capture.set(cv::CAP_PROP_AUTOFOCUS, 0.0)) {
			std::cerr << "[rectai-tracker] Warning: failed to disable autofocus on this camera/backend." << std::endl;
		} else {
			std::cout << "[rectai-tracker] Continuous autofocus disabled via --no-autofocus." << std::endl;
		}
	}

	if (requestedResolution.has_value()) {
		(void)capture.set(cv::CAP_PROP_FRAME_WIDTH, requestedResolution->width);
		(void)capture.set(cv::CAP_PROP_FRAME_HEIGHT, requestedResolution->height);
	}

	if (requestedFps.has_value()) {
		(void)capture.set(cv::CAP_PROP_FPS, static_cast<double>(*requestedFps));
	}

	// Force the backend to actually start the stream with the
	// requested settings before initialising the tracker. With
	// some backends (notably GStreamer) the pipeline may fail to
	// start even though isOpened() returns true, leading to
	// width/height=0 and confusing errors later.
	{
		cv::Mat probeFrame;
		if (!capture.read(probeFrame) || probeFrame.empty()) {
			std::cerr << "[rectai-tracker] Failed to start camera stream on index "
			          << cameraIndex
			          << " with the requested settings (format/resolution/fps)." << std::endl;
			return 1;
		}
	}

	std::cout << "[rectai-tracker] Camera opened. Press Ctrl+C to exit." << std::endl;

	// OSC sender towards the JUCE core (localhost:3333 by default).
	OscSender oscSender{"127.0.0.1", 3333};
	if (!oscSender.isOk()) {
		std::cerr << "[rectai-tracker] OSC sender is not available."
				  << " Core will not receive tracking updates." << std::endl;
	}

	rectai::tracker::TuioOutputMode outputMode = rectai::tracker::TuioOutputMode::LegacyOsc;
	if (oscSender.isOk()) {
		if (forceOsc) {
			std::cout << "[rectai-tracker] Forcing legacy OSC output via --osc" << std::endl;
			outputMode = rectai::tracker::TuioOutputMode::LegacyOsc;
		} else {
			outputMode = rectai::tracker::TuioOutputMode::Tuio11;
			// Fire a best-effort TUIO hello so the core can log
			// tracker identity and version. We no longer wait for an ACK.
			if (!oscSender.sendHelloTuio11("rectai-tracker", "0.1.0")) {
				std::cerr << "[rectai-tracker] Failed to send /tuio/hello announcement" << std::endl;
			}
		}
	}

	rectai::tracker::TrackerEngine trackerEngine;
	if (filterIndex.has_value()) {
		trackerEngine.setOnlySingleFilterByIndex(true, *filterIndex);
		std::cout << "[rectai-tracker] Using single-threshold mode via --filter" << std::endl;
	} else if (onlyFilter) {
		trackerEngine.setOnlySingleFilter(true);
		std::cout << "[rectai-tracker] Using single-threshold mode via --only-filter" << std::endl;
	}
	rectai::tracker::TrackerState trackerState;
	rectai::tracker::TrackedObjectList lastStableObjectsForDebug;
	std::unordered_map<int, rectai::tracker::TrackedObject> lastStableById;
	std::string initError;
	int tuioFrameSeq = 0;
	int tuioQuietFrameModuloCounter = 0;

	// Require several consecutive detections of the same fiducial ID,
	// with only small spatial jitter between frames, before considering
	// it a valid object. This helps to filter out spurious and unstable
	// detections coming from the camera feed.
	std::unordered_map<int, int> consecutiveDetectionsById;
	std::unordered_map<int, std::pair<float, float>> lastPositionById;

	// Adaptive processing rate: when we have not seen any fiducials
	// for a long time, we can reduce how often we run the expensive
	// tracking pipeline. Once we detect something again, we go back
	// to processing every frame.
	int framesWithoutDetections = 0;
	bool lowProcessingRate = false;
	double lastProcessingTimeSec = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();
	double lastStableUpdateSec = 0.0;

	if (mode == RunMode::Live) {
		int width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
		int height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
		const double fps = capture.get(cv::CAP_PROP_FPS);
		if (!trackerEngine.initialise(cameraIndex, width, height, initError, !noDownscale)) {
			std::cerr << "[rectai-tracker] Failed to initialise TrackerEngine: "
					  << initError << std::endl;
			return 1;
		}
		std::cout << "[rectai-tracker] Starting engine with resolution="
				  << width << "x" << height;
		if (fps > 0.0) {
			std::cout << " @ " << fps << " FPS";
		}
		std::cout << std::endl;
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

		const double nowTicks = static_cast<double>(cv::getTickCount());
		const double nowSec = nowTicks / cv::getTickFrequency();

		if (mode == RunMode::Synthetic) {
			if (oscSender.isOk()) {
				phase += 0.01F;
				if (phase > 2.0F * static_cast<float>(M_PI)) {
					phase -= 2.0F * static_cast<float>(M_PI);
				}

				const float x = 0.5F + 0.3F * std::sin(phase);
				const float y = 0.5F;
				const float angleDegrees = 0.0F;

				if (outputMode == rectai::tracker::TuioOutputMode::Tuio11) {
					const float angleRad = angleDegrees *
						( static_cast<float>(M_PI) / 180.0F );
					std::vector<OscSender::Message> messages;
					messages.reserve(3);
					messages.push_back(oscSender.buildTuio2DobjAlive({1}));
					messages.push_back(oscSender.buildTuio2DobjSet(1, 1, x, y, angleRad));
					messages.push_back(oscSender.buildTuio2DobjFseq(tuioFrameSeq++));
					(void)oscSender.sendBundle(messages);
				} else {
					std::vector<OscSender::Message> messages;
					messages.reserve(1);
					messages.push_back(oscSender.buildRectaiObject(1, "osc1", x, y, angleDegrees));
					(void)oscSender.sendBundle(messages);
				}
			}
		} else {
			rectai::tracker::TrackedObjectList objects;
			bool processedThisFrame = false;

			// Decide whether to run the tracker on this frame based on
			// the current processing rate mode.
			if (!lowProcessingRate) {
				if (debugView) {
					objects = trackerEngine.processFrame(frame, debugFrame);
				} else {
					objects = trackerEngine.processFrame(frame);
				}
				processedThisFrame = true;
				lastProcessingTimeSec = nowSec;
			} else {
				// Low-rate mode: only process a subset of frames.
				constexpr double kLowRateFps = 15.0;
				const double minIntervalSec = 1.0 / kLowRateFps;
				if (nowSec - lastProcessingTimeSec >= minIntervalSec) {
					if (debugView) {
						objects = trackerEngine.processFrame(frame, debugFrame);
					} else {
						objects = trackerEngine.processFrame(frame);
					}
					processedThisFrame = true;
					lastProcessingTimeSec = nowSec;
				}
			}

			if (!processedThisFrame) {
				// Skip updates for this camera frame; keep last known
				// stable objects and wait for the next processed one.
				continue;
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

			// Maintain a simple heuristic of how many processed frames
			// we have gone without seeing any objects. If we see nothing
			// for a long time, we throttle the processing rate; any new
			// detection brings us back to full rate.
			if (objects.empty()) {
				++framesWithoutDetections;
			} else {
				framesWithoutDetections = 0;
			}

			if (!lowProcessingRate && framesWithoutDetections >= 100) {
				lowProcessingRate = true;
				std::cout << "[rectai-tracker] No fiducials detected for 100 frames, throttling processing to ~15 FPS." << std::endl;
			} else if (lowProcessingRate && !objects.empty()) {
				lowProcessingRate = false;
				std::cout << "[rectai-tracker] Fiducials detected again, resuming full-rate processing." << std::endl;
			}

			// Update lifetime tracking only with stable objects so that
			// removals are also consistent with the stability threshold.
			trackerState.update(stableObjects);
			lastStableObjectsForDebug = stableObjects;

			// Estimate per-object velocities (normalised units per second)
			// and angular velocity from the last stable sample.
			const double dtSec = (lastStableUpdateSec > 0.0)
			                        ? (nowSec - lastStableUpdateSec)
			                        : 0.0;
			lastStableUpdateSec = nowSec;

			if (oscSender.isOk()) {
				bool shouldSendTuioThisFrame = true;
				if (outputMode == rectai::tracker::TuioOutputMode::Tuio11) {
					// When running in low processing rate mode (~15 FPS) and
					// no tangible objects are detected, reduce the frequency
					// of TUIO events to ~5 Hz by only sending one out of
					// every three processed frames.
					if (lowProcessingRate && stableObjects.empty()) {
						++tuioQuietFrameModuloCounter;
						if (tuioQuietFrameModuloCounter % 3 != 0) {
							shouldSendTuioThisFrame = false;
						}
					} else {
						tuioQuietFrameModuloCounter = 0;
					}
				}

				if (outputMode == rectai::tracker::TuioOutputMode::Tuio11 && shouldSendTuioThisFrame) {
					std::vector<OscSender::Message> messages;
					messages.reserve(stableObjects.size() + 2U);

					std::vector<std::int32_t> aliveIds;
					aliveIds.reserve(stableObjects.size());
					for (const auto& obj : stableObjects) {
						aliveIds.push_back(obj.id);
					}
					messages.push_back(oscSender.buildTuio2DobjAlive(aliveIds));
					for (const auto& obj : stableObjects) {
						const float angleRad = obj.angle_rad;

						// Map physical fiducial ID to logical module ID
						// and use that as the TUIO symbol ID.
						const std::string logicalIdStr = mapLogicalId(obj.id);
						int symbolId = obj.id;
						try {
							if (!logicalIdStr.empty()) {
								symbolId = std::stoi(logicalIdStr);
							}
						} catch (const std::exception&) {
							// Fallback: keep the physical id if parsing fails.
						}

						// Approximate velocities using the difference between
						// the current and last stable pose for this id.
						float vx = 0.0F;
						float vy = 0.0F;
						float omega = 0.0F;
						if (dtSec > 0.0) {
							if (const auto it = lastStableById.find(obj.id);
							    it != lastStableById.end()) {
								const auto& prev = it->second;
								vx = (obj.x_norm - prev.x_norm) /
								     static_cast<float>(dtSec);
								vy = (obj.y_norm - prev.y_norm) /
								     static_cast<float>(dtSec);
								float dAngle = obj.angle_rad - prev.angle_rad;
								// Wrap angular difference into [-pi, pi] to
								// avoid spikes on wrap-around.
								const float pi = static_cast<float>(M_PI);
								if (dAngle > pi) {
									dAngle -= 2.0F * pi;
								} else if (dAngle < -pi) {
									dAngle += 2.0F * pi;
								}
								omega = dAngle / static_cast<float>(dtSec);
							}
						}

						messages.push_back(
						    oscSender.buildTuio2DobjSet(
						        obj.id, symbolId, obj.x_norm, obj.y_norm,
						        angleRad, vx, vy, omega));
					}
					messages.push_back(oscSender.buildTuio2DobjFseq(tuioFrameSeq++));
					(void)oscSender.sendBundle(messages);

				} else if (outputMode == rectai::tracker::TuioOutputMode::LegacyOsc) {
					std::vector<OscSender::Message> messages;
					messages.reserve(stableObjects.size() + 8U);

					for (const auto& obj : stableObjects) {
						const std::string logicalId = mapLogicalId(obj.id);
						const float angleDegrees = obj.angle_rad *
							(180.0F / static_cast<float>(M_PI));
						std::cout << "[rectai-tracker] fiducial " << obj.id
								<< " -> logicalId=" << logicalId
								<< " x=" << obj.x_norm
								<< " y=" << obj.y_norm
								<< " angle_deg=" << angleDegrees << '\n';
						messages.push_back(
						    oscSender.buildRectaiObject(
						        obj.id, logicalId, obj.x_norm, obj.y_norm,
						        angleDegrees));
					}

					// Any id that has been missing for several frames is treated as
					// removed from the musical area, similar to a TUIO remove event.
					const auto removedIds = trackerState.collectRemovals(15);
					for (const int id : removedIds) {
						std::cout << "[rectai-tracker] fiducial " << id
							      << " removed from tracking" << '\n';
						messages.push_back(oscSender.buildRectaiRemove(id));
					}

					if (!messages.empty()) {
						(void)oscSender.sendBundle(messages);
					}
				}
			}

			// Update last stable poses after emitting messages so that
			// the next frame can use them to compute velocities.
			lastStableById.clear();
			for (const auto& obj : stableObjects) {
				lastStableById[obj.id] = obj;
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
