#include "TrackingOscReceiver.h"

#include <utility>

namespace {
constexpr int kDefaultPort = 3333;  // Aligned with common TUIO setups.
}

TrackingOscReceiver::TrackingOscReceiver(rectai::Scene& scene, const int port)
    : scene_(scene)
{
    addListener(this, "/rectai/object");
    addListener(this, "/rectai/remove");

    const auto actualPort = port > 0 ? port : kDefaultPort;
    if (!connect(actualPort)) {
        juce::Logger::writeToLog(
            "[rectai-core] Failed to bind OSC receiver on port " +
            juce::String(actualPort));
    } else {
        juce::Logger::writeToLog(
            "[rectai-core] OSC receiver listening on port " +
            juce::String(actualPort));
    }
}

TrackingOscReceiver::~TrackingOscReceiver()
{
    disconnect();
}

void TrackingOscReceiver::oscMessageReceived(const juce::OSCMessage& message)
{
    const auto address = message.getAddressPattern().toString();

    if (address == "/rectai/object") {
        handleObjectMessage(message);
    } else if (address == "/rectai/remove") {
        handleRemoveMessage(message);
    }
}

void TrackingOscReceiver::handleObjectMessage(const juce::OSCMessage& message)
{
    // Expected: int32 trackingId, string logicalId, float x, float y,
    // float angleDegrees.
    if (message.size() < 5) {
        juce::Logger::writeToLog(
            "[rectai-core] /rectai/object: invalid argument count");
        return;
    }

    if (!message[0].isInt32() || !message[1].isString() ||
        !message[2].isFloat32() || !message[3].isFloat32() ||
        !message[4].isFloat32()) {
        juce::Logger::writeToLog(
            "[rectai-core] /rectai/object: unexpected argument types");
        return;
    }

    const auto trackingId = static_cast<std::int64_t>(message[0].getInt32());
    const auto logicalId = message[1].getString().toStdString();

    // The tracker sends normalised coordinates in [0,1] relative to the
    // camera frame. Convert them to the centre-origin table coordinate
    // system used by the Scene and UI, where the table centre is (0,0) and
    // the table border lies at radius 1 (left edge X = -1, right edge X = +1).
    const auto xNorm = message[2].getFloat32();
    const auto yNorm = message[3].getFloat32();
    const float x = 2.0F * xNorm - 1.0F;
    const float y = 2.0F * yNorm - 1.0F;
    const auto angleDegrees = message[4].getFloat32();

    constexpr float kPi = juce::MathConstants<float>::pi;
    const float angleRadians = angleDegrees * (kPi / 180.0F);

    rectai::ObjectInstance instance(trackingId, logicalId, x, y,
                                    angleRadians);
    scene_.UpsertObject(instance);
}

void TrackingOscReceiver::handleRemoveMessage(const juce::OSCMessage& message)
{
    // Expected: int32 trackingId.
    if (message.size() < 1 || !message[0].isInt32()) {
        juce::Logger::writeToLog(
            "[rectai-core] /rectai/remove: invalid arguments");
        return;
    }

    const auto trackingId = static_cast<std::int64_t>(message[0].getInt32());

    // Instead of removing the object from the scene entirely, mark
    // its associated module as docked again so that it returns to the
    // dock/toolbar area when the fiducial disappears from tracking.
    const auto& objects = scene_.objects();
    const auto it = objects.find(trackingId);
    if (it == objects.end()) {
        return;
    }

    const auto& obj = it->second;
    rectai::ObjectInstance dockedInstance(
        trackingId, obj.logical_id(), obj.x(), obj.y(),
        obj.angle_radians(), /*docked=*/true);
    scene_.UpsertObject(dockedInstance);
}
