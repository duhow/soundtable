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
    // Expected: int32 trackingId, string logicalId, float x, float y, float angle.
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
    const auto x = message[2].getFloat32();
    const auto y = message[3].getFloat32();
    const auto angle = message[4].getFloat32();

    rectai::ObjectInstance instance(trackingId, logicalId, x, y, angle);
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
    scene_.RemoveObject(trackingId);
}
