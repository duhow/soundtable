#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include <juce_osc/juce_osc.h>

#include "core/Scene.h"

// Simple OSC-based bridge to update the Scene from tracking messages.
// For now this uses a very small custom protocol:
//   - /rectai/object  (int32 trackingId, string logicalId,
//                      float x, float y, float angleDegrees)
//       => upsert ObjectInstance in the Scene. The incoming angle is
//          expressed in degrees in the range [0, 360] and is converted
//          internally to radians for the Scene model.
//   - /rectai/remove  (int32 trackingId)
//       => remove ObjectInstance from the Scene
//
// This receiver also supports standard OSC bundles as used by
// reacTIVision and TUIO Simulator. Incoming bundles are recursively
// flattened so that each contained OSC message is dispatched through
// the same handlers as standalone messages.
class TrackingOscReceiver : private juce::OSCReceiver,
                            private juce::OSCReceiver::Listener<
                                juce::OSCReceiver::MessageLoopCallback> {
public:
    enum class ActivityKind { kOsc = 0, kTuio = 1 };

    TrackingOscReceiver(rectai::Scene& scene, int port);
    ~TrackingOscReceiver() override;

    // Register optional callbacks to mirror TUIO 2D cursor events into
    // higher-level UI gestures. Coordinates are provided normalizadas
    // en [0,1] sobre el área completa del componente (0 = borde
    // izquierdo/superior, 1 = borde derecho/inferior), de forma que la
    // UI pueda proyectarlas directamente a píxeles relativos a
    // getLocalBounds().
    void setTuioCursorCallbacks(
        std::function<void(float normX, float normY,
                           std::int32_t sessionId)> onDown,
        std::function<void(float normX, float normY,
                           std::int32_t sessionId)> onMove,
        std::function<void(std::int32_t sessionId)> onUp);

    // Optional callback invoked whenever an incoming OSC message is
    // successfully classified as either proprietary rectai OSC traffic
    // (ActivityKind::kOsc) or standard TUIO traffic (ActivityKind::kTuio).
    // This is used by the UI layer to drive lightweight "traffic"
    // indicators without coupling visual code into the receiver.
    void setActivityCallback(
        std::function<void(ActivityKind kind)> onActivity);

private:
    void oscMessageReceived(const juce::OSCMessage& message) override;
    void oscBundleReceived(const juce::OSCBundle& bundle) override;

    void handleObjectMessage(const juce::OSCMessage& message);
    void handleRemoveMessage(const juce::OSCMessage& message);
    void handleTuioHelloMessage(const juce::OSCMessage& message);
    void handleTuio2DobjMessage(const juce::OSCMessage& message);
    void handleTuio2DcurMessage(const juce::OSCMessage& message);

    void writeLog(const juce::String& text);
    void writeLog(const juce::OSCMessage& message, const juce::String& text);

    void notifyActivity(ActivityKind kind);

    rectai::Scene& scene_;

    // Track the current set of alive TUIO session ids in order to
    // detect removals and mirror the /rectai/remove behaviour.
    std::unordered_set<std::int32_t> tuioAliveSessionIds_;

    // Track active TUIO cursor session ids and optional UI callbacks
    // used to mirror 2Dcur gestures into MainComponent.
    std::unordered_set<std::int32_t> tuioCursorSessionIds_;
    std::unordered_map<std::int32_t, std::pair<float, float>>
        tuioCursorPositions_;
    std::function<void(float, float, std::int32_t)> tuioCursorDownCallback_;
    std::function<void(float, float, std::int32_t)> tuioCursorMoveCallback_;
    std::function<void(std::int32_t)> tuioCursorUpCallback_;

    std::function<void(ActivityKind)> activityCallback_;
};
