#include "TrackingOscReceiver.h"

#include <utility>

namespace {
constexpr int kDefaultPort = 3333;  // Aligned with common TUIO setups.
}  // namespace

TrackingOscReceiver::TrackingOscReceiver(soundtable::Scene& scene, const int port)
    : scene_(scene)
{
    // Register as a generic OSC listener; address-based dispatch is
    // handled manually in oscMessageReceived so that we can also
    // support messages arriving inside OSC bundles.
    addListener(this);

    const auto actualPort = port > 0 ? port : kDefaultPort;
    if (!connect(actualPort)) {
        juce::Logger::writeToLog(
            "[soundtable-core] Failed to bind OSC receiver on port " +
            juce::String(actualPort));
    } else {
        juce::Logger::writeToLog(
            "[soundtable-core] OSC receiver listening on port " +
            juce::String(actualPort));
    }

}

TrackingOscReceiver::~TrackingOscReceiver()
{
    disconnect();
}

void TrackingOscReceiver::setTuioCursorCallbacks(
    std::function<void(float, float, std::int32_t)> onDown,
    std::function<void(float, float, std::int32_t)> onMove,
    std::function<void(std::int32_t)> onUp)
{
    tuioCursorDownCallback_ = std::move(onDown);
    tuioCursorMoveCallback_ = std::move(onMove);
    tuioCursorUpCallback_ = std::move(onUp);
}

void TrackingOscReceiver::writeLog(
    const juce::String& text)
{
#if !defined(NDEBUG)
    juce::Logger::writeToLog("[soundtable-core] " + text);
#endif
    return;
}

void TrackingOscReceiver::writeLog(
    const juce::OSCMessage& message,
    const juce::String& text)
{
#if !defined(NDEBUG)
    const auto address = message.getAddressPattern().toString();
    juce::Logger::writeToLog(juce::String("[soundtable-core] ") + address + ": " + text);
#endif
    return;
}

void TrackingOscReceiver::setActivityCallback(
    std::function<void(ActivityKind)> onActivity)
{
    activityCallback_ = std::move(onActivity);
}

void TrackingOscReceiver::notifyActivity(const ActivityKind kind)
{
    if (activityCallback_) {
        activityCallback_(kind);
    }
}

void TrackingOscReceiver::oscMessageReceived(const juce::OSCMessage& message)
{
    const auto address = message.getAddressPattern().toString();

    if (address.startsWith("/soundtable/")) {
        notifyActivity(ActivityKind::kOsc);
    } else if (address.startsWith("/tuio/")) {
        notifyActivity(ActivityKind::kTuio);
    }

    if (address == "/soundtable/object") {
        handleObjectMessage(message);
    } else if (address == "/soundtable/remove") {
        handleRemoveMessage(message);
    } else if (address == "/tuio/hello") {
        handleTuioHelloMessage(message);
    } else if (address == "/tuio/2Dobj") {
        handleTuio2DobjMessage(message);
    } else if (address == "/tuio/2Dcur") {
        handleTuio2DcurMessage(message);
    } else {
        juce::String debug = "OSC message on unexpected address '" +
                            address + "' (" + juce::String(message.size()) +
                            " args)";
        writeLog(debug);
    }
}

void TrackingOscReceiver::oscBundleReceived(const juce::OSCBundle& bundle)
{
    // Flatten bundles recursively so that each contained OSC message
    // is dispatched through the same handler as standalone messages.
    for (int i = 0; i < bundle.size(); ++i) {
        const auto& element = bundle[i];

        if (element.isMessage()) {
            oscMessageReceived(element.getMessage());
        } else if (element.isBundle()) {
            oscBundleReceived(element.getBundle());
        }
    }
}

void TrackingOscReceiver::handleObjectMessage(const juce::OSCMessage& message)
{
    // Expected: int32 trackingId, string logicalId, float x, float y,
    // float angleDegrees.
    if (message.size() < 5) {
        writeLog(message, "invalid argument count");
        return;
    }

    if (!message[0].isInt32() || !message[1].isString() ||
        !message[2].isFloat32() || !message[3].isFloat32() ||
        !message[4].isFloat32()) {
        writeLog(message, "unexpected argument types");
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

    soundtable::ObjectInstance instance(trackingId, logicalId, x, y,
                                    angleRadians);
    scene_.UpsertObject(instance);
}

void TrackingOscReceiver::handleRemoveMessage(const juce::OSCMessage& message)
{
    // Expected: int32 trackingId.
    if (message.size() < 1 || !message[0].isInt32()) {
        writeLog(message, "invalid arguments");
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
    soundtable::ObjectInstance dockedInstance(
        trackingId, obj.logical_id(), obj.x(), obj.y(),
        obj.angle_radians(), /*docked=*/true);
    scene_.UpsertObject(dockedInstance);
}

void TrackingOscReceiver::handleTuioHelloMessage(
    const juce::OSCMessage& message)
{
    // Expected: string trackerId, string tuioVersion, string trackerVersion.
    if (message.size() < 2 || !message[0].isString() ||
        !message[1].isString()) {
        writeLog(message, "unexpected argument types");
        return;
    }

    const auto trackerId = message[0].getString();
    const auto tuioVersion = message[1].getString();
    juce::String clientVersion{"unknown"};
    if (message.size() >= 3 && message[2].isString()) {
        clientVersion = message[2].getString();
    }

    juce::Logger::writeToLog(
        "[soundtable-core] TUIO hello from " + trackerId +
        " (client " + clientVersion + ", TUIO " + tuioVersion +
        ", via localhost:" + juce::String(kDefaultPort) + ")");
}

void TrackingOscReceiver::handleTuio2DobjMessage(
    const juce::OSCMessage& message)
{
    // TUIO 2Dobj profile:
    //   /tuio/2Dobj set s_id i_id x y a X Y A m r
    //   /tuio/2Dobj alive s_id1 s_id2 ...
    //   /tuio/2Dobj fseq frameSeq
    if (message.size() < 1 || !message[0].isString()) {
        writeLog(message, "missing command string");
        return;
    }

    const auto command = message[0].getString();

    writeLog(message, "command '" + command + "' with " + juce::String(message.size()) + " args");

    if (command == "set") {
        // Expect at least: set, s_id (int32), i_id (int32), x, y, a (floats).
        // Velocity components X, Y, A are optional and default to 0 when
        // omitted by the sender.
        if (message.size() < 6 || !message[1].isInt32() ||
            !message[2].isInt32() || !message[3].isFloat32() ||
            !message[4].isFloat32() || !message[5].isFloat32()) {
            writeLog(message, "set: invalid arguments");
            return;
        }

        const auto sessionId = message[1].getInt32();
        const auto symbolId = message[2].getInt32();
        const auto xNorm = message[3].getFloat32();
        const auto yNorm = message[4].getFloat32();
        const auto angleRadians = message[5].getFloat32();

        float velocityX = 0.0F;
        float velocityY = 0.0F;
        float angularVelocity = 0.0F;
        if (message.size() >= 9 && message[6].isFloat32() &&
            message[7].isFloat32() && message[8].isFloat32()) {
            velocityX = message[6].getFloat32();
            velocityY = message[7].getFloat32();
            angularVelocity = message[8].getFloat32();
        }

        const float x = 2.0F * xNorm - 1.0F;
        const float y = 2.0F * yNorm - 1.0F;

        juce::Logger::writeToLog(
            "[soundtable-core] /tuio/2Dobj set: s_id=" +
            juce::String(sessionId) + ", i_id=" +
            juce::String(symbolId) + ", xNorm=" +
            juce::String(xNorm, 3) + ", yNorm=" +
            juce::String(yNorm, 3) + ", angleRad=" +
            juce::String(angleRadians, 3));

        // The tracker now sends the logical module id as the TUIO
        // symbol id, so we can derive the Scene logical_id string
        // directly from it.
        const auto logicalId = std::to_string(symbolId);

        soundtable::ObjectInstance instance(
            static_cast<std::int64_t>(sessionId), logicalId, x, y,
            angleRadians, velocityX, velocityY, angularVelocity);
        scene_.UpsertObject(instance);
        tuioAliveSessionIds_.insert(sessionId);
    } else if (command == "alive") {
        // Build the new alive set of session ids.
        std::unordered_set<std::int32_t> newAlive;
        for (int i = 1; i < message.size(); ++i) {
            if (message[i].isInt32()) {
                newAlive.insert(message[i].getInt32());
            }
        }

        if (!newAlive.empty()) {
            juce::String ids;
            bool first = true;
            for (const auto id : newAlive) {
                if (!first) {
                    ids += ",";
                }
                ids += juce::String(id);
                first = false;
            }
            writeLog(message, "alive: " + ids);
        }

        // Any id that was previously alive but is not in the new set
        // is treated as removed and we dock its associated module.
        for (const auto previousId : tuioAliveSessionIds_) {
            if (newAlive.find(previousId) == newAlive.end()) {
                const auto& objects = scene_.objects();
                const auto it = objects.find(previousId);
                if (it != objects.end()) {
                    const auto& obj = it->second;
                    soundtable::ObjectInstance dockedInstance(
                        static_cast<std::int64_t>(previousId),
                        obj.logical_id(), obj.x(), obj.y(),
                        obj.angle_radians(), /*docked=*/true);
                    scene_.UpsertObject(dockedInstance);
                }
            }
        }

        tuioAliveSessionIds_ = std::move(newAlive);
    } else if (command == "fseq") {
        // Frame sequence marker; we do not need it yet.
        return;
    }
}

void TrackingOscReceiver::handleTuio2DcurMessage(
    const juce::OSCMessage& message)
{
    // TUIO 2Dcur profile:
    //   /tuio/2Dcur set s_id x y X Y m
    //   /tuio/2Dcur alive s_id1 s_id2 ...
    //   /tuio/2Dcur fseq frameSeq
    if (message.size() < 1 || !message[0].isString()) {
        juce::Logger::writeToLog(
            "[soundtable-core] /tuio/2Dcur: missing command string");
        return;
    }

    const auto command = message[0].getString();

    juce::Logger::writeToLog("[soundtable-core] /tuio/2Dcur command '" +
                             command + "' with " +
                             juce::String(message.size()) + " args");

    if (command == "set") {
        // Expect at least: set, s_id (int32), x, y (floats).
        if (message.size() < 4 || !message[1].isInt32() ||
            !message[2].isFloat32() || !message[3].isFloat32()) {
            writeLog(message, "set: invalid arguments");
            return;
        }

        const auto sessionId = message[1].getInt32();
        const auto xNorm = message[2].getFloat32();
        const auto yNorm = message[3].getFloat32();

        // Store positions normalised en [0,1] sobre el área completa
        // del componente. La proyección a píxeles se hace en
        // MainComponent, usando getLocalBounds().
        const float normX = xNorm;
        const float normY = yNorm;

        const bool isNew =
            (tuioCursorPositions_.find(sessionId) ==
             tuioCursorPositions_.end());

        tuioCursorPositions_[sessionId] = {normX, normY};

        juce::Logger::writeToLog(
            "[soundtable-core] /tuio/2Dcur set: s_id=" +
            juce::String(sessionId) + ", xNorm=" +
            juce::String(normX, 3) + ", yNorm=" +
            juce::String(normY, 3));

        if (isNew) {
            tuioCursorSessionIds_.insert(sessionId);
            if (tuioCursorDownCallback_) {
                tuioCursorDownCallback_(normX, normY, sessionId);
            }
        } else {
            if (tuioCursorMoveCallback_) {
                tuioCursorMoveCallback_(normX, normY, sessionId);
            }
        }
    } else if (command == "alive") {
        // Build the new alive set of cursor session ids.
        std::unordered_set<std::int32_t> newAlive;
        for (int i = 1; i < message.size(); ++i) {
            if (message[i].isInt32()) {
                newAlive.insert(message[i].getInt32());
            }
        }

        if (!newAlive.empty()) {
            juce::String ids;
            bool first = true;
            for (const auto id : newAlive) {
                if (!first) {
                    ids += ",";
                }
                ids += juce::String(id);
                first = false;
            }
            writeLog(message, "alive: " + ids);
        }

        // Any id that was previously alive but is not in the new set
        // is treated as a pointer up.
        for (const auto previousId : tuioCursorSessionIds_) {
            if (newAlive.find(previousId) == newAlive.end()) {
                if (tuioCursorUpCallback_) {
                    tuioCursorUpCallback_(previousId);
                }
                tuioCursorPositions_.erase(previousId);
            }
        }

        tuioCursorSessionIds_ = std::move(newAlive);
    } else if (command == "fseq") {
        // Frame sequence marker; currently unused.
        return;
    }
}
