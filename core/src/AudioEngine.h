#pragma once

#include <atomic>

#include <juce_audio_devices/juce_audio_devices.h>
#include <juce_audio_basics/juce_audio_basics.h>

// Minimal audio engine for the RectaiTable core.
//
// Responsibilities (for now):
//   - Own a JUCE AudioDeviceManager.
//   - Produce a simple sine tone so that the audio pipeline is live.
//
// Future extensions:
//   - Map rectai::Scene / AudioModule metadata to an AudioProcessorGraph.
//   - Control parameters from the UI and tracking state.
class AudioEngine : public juce::AudioIODeviceCallback {
public:
    AudioEngine();
    ~AudioEngine() override;

    // Maximum number of independent generator voices mixed by the engine.
    static constexpr int kMaxVoices = 16;

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

    // Control API (voice 0 convenience wrappers)
    void setFrequency(double frequency);
    void setLevel(float level);

    // Multi-voice control: index in [0, kMaxVoices).
    void setVoice(int index, double frequency, float level);

private:
    juce::AudioDeviceManager deviceManager_;

    double sampleRate_{44100.0};

    struct Voice {
        std::atomic<double> frequency{0.0};
        std::atomic<float> level{0.0F};
    };

    Voice voices_[kMaxVoices];
    std::atomic<int> numVoices_{0};

    // Phase per voice, only touched on the audio thread.
    double phases_[kMaxVoices]{};
};
