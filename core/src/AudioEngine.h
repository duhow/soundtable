#pragma once

#include <atomic>

#include <juce_audio_devices/juce_audio_devices.h>
#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_dsp/juce_dsp.h>

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

    // Number of mono samples kept in the rolling buffer used for
    // waveform visualisation. At 44.1 kHz, 4096 samples correspond
    // to ~93 ms of audio, enough to extract a ~50 ms window.
    static constexpr int kWaveformHistorySize = 4096;

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

    // Copies a downsampled snapshot of the recent mono output mix into
    // `dst`, representing approximately `windowSeconds` of audio
    // (clamped to the internal history size). The snapshot is safe to
    // call from the GUI thread without blocking the audio callback.
    void getWaveformSnapshot(float* dst, int numPoints,
                             double windowSeconds);

    // Copies a downsampled snapshot of the recent mono output for a
    // specific voice into `dst`, representing approximately
    // `windowSeconds` of audio. If the voice index is out of range, or
    // there is not enough history yet, the destination is cleared.
    void getVoiceWaveformSnapshot(int voiceIndex, float* dst,
                                  int numPoints, double windowSeconds);

    // Configure filter parameters for a given voice. This can be
    // called from the UI thread; the audio thread will consume the
    // updated coefficients on the next callback.
    void setVoiceFilter(int index, int mode, double cutoffHz, float q);

private:
    juce::AudioDeviceManager deviceManager_;

    double sampleRate_{44100.0};

    struct Voice {
        std::atomic<double> frequency{0.0};
        std::atomic<float> level{0.0F};
        // Filter parameters controlled from the Scene. Mode is an
        // integer selector (0 = bypass, 1 = low-pass, 2 = band-pass,
        // 3 = high-pass).
        std::atomic<int> filterMode{0};
        std::atomic<double> filterCutoffHz{0.0};
        std::atomic<float> filterQ{0.7071F};
    };

    Voice voices_[kMaxVoices];
    std::atomic<int> numVoices_{0};

    // Phase per voice, only touched on the audio thread.
    double phases_[kMaxVoices]{};

    // Per-voice state-variable filters implemented using JUCE's DSP
    // module for improved stability and sound quality.
    juce::dsp::StateVariableTPTFilter<float> filters_[kMaxVoices]{};

    // Rolling mono mix buffer for visualisation.
    float waveformBuffer_[kWaveformHistorySize]{};

    // Per-voice rolling buffers used to render per-module waveforms in
    // the UI. Each buffer shares the same write index as the global
    // mix buffer so that all curves are time-aligned.
    float voiceWaveformBuffer_[kMaxVoices][kWaveformHistorySize]{};
    std::atomic<int> waveformWriteIndex_{0};
};
