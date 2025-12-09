#pragma once

#include <juce_osc/juce_osc.h>

#include "core/Scene.h"

// Simple OSC-based bridge to update the Scene from tracking messages.
// For now this uses a very small custom protocol:
//   - /rectai/object  (int32 trackingId, string logicalId,
//                      float x, float y, float angleRadians)
//       => upsert ObjectInstance in the Scene
//   - /rectai/remove  (int32 trackingId)
//       => remove ObjectInstance from the Scene
class TrackingOscReceiver : private juce::OSCReceiver,
                            private juce::OSCReceiver::ListenerWithOSCAddress<
                                juce::OSCReceiver::MessageLoopCallback> {
public:
    TrackingOscReceiver(rectai::Scene& scene, int port);
    ~TrackingOscReceiver() override;

private:
    void oscMessageReceived(const juce::OSCMessage& message) override;

    void handleObjectMessage(const juce::OSCMessage& message);
    void handleRemoveMessage(const juce::OSCMessage& message);

    rectai::Scene& scene_;
};
