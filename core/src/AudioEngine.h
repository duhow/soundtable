#pragma once

#include <juce_audio_devices/juce_audio_devices.h>
#include <juce_audio_basics/juce_audio_basics.h>

// Minimal audio engine for the RectaiTable core.
//
// Responsibilities (for now):
//   - Own a JUCE AudioDeviceManager.
//   - Produce a simple sine tone so that the audio pipeline is live.
//
// Future extensions:
//   - Map rectai::Scene / ModuleKind to an AudioProcessorGraph.
//   - Control parameters from the UI and tracking state.
class AudioEngine : public juce::AudioIODeviceCallback {
public:
    AudioEngine();
    ~AudioEngine() override;

    // juce::AudioIODeviceCallback
    void audioDeviceAboutToStart(juce::AudioIODevice* device) override;
    void audioDeviceStopped() override;
    void audioDeviceIOCallbackWithContext(
        const float* const* inputChannelData,
        int numInputChannels,
        float* const* outputChannelData,
        int numOutputChannels,
        int numSamples,
        const juce::AudioIODeviceCallbackContext& context) override;

private:
    juce::AudioDeviceManager deviceManager_;

    double sampleRate_{44100.0};
    double currentAngle_{0.0};
    double angleDelta_{0.0};
    float level_{0.1F};  // Output level to avoid clipping.
};
