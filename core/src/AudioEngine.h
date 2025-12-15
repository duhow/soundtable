#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include <juce_audio_devices/juce_audio_devices.h>
#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_dsp/juce_dsp.h>

namespace rectai {
class SampleplaySynth;
}  // namespace rectai

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

    // Returns true when the audio device could not provide any
    // output channels during initialisation (for example when the
    // system has no active audio device). The UI can use this to
    // reflect a degraded audio state.
    [[nodiscard]] bool hasNoOutputChannels() const noexcept
    {
        return noOutputChannels_;
    }

    // Returns true when there was any error while initialising the
    // audio device manager. This includes the "no channels" case but
    // also other failures (invalid device configuration, missing
    // backend, etc.).
    [[nodiscard]] bool hasInitialisationError() const noexcept
    {
        return initError_;
    }

    // Control API (voice 0 convenience wrappers)
    void setFrequency(double frequency);
    void setLevel(float level);

    // Multi-voice control: index in [0, kMaxVoices).
    void setVoice(int index, double frequency, float level);

    // Selects the waveform shape for a given voice. The waveform
    // index is clamped to the supported range [0, 3].
    void setVoiceWaveform(int index, int waveformIndex);

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

    // Copies a downsampled snapshot of the recent mono output from
    // the Sampleplay (SoundFont) path into `dst`, representing
    // approximately `windowSeconds` of audio. This is used so that
    // Sampleplay modules can display their own waveform on lines even
    // though they do not occupy an AudioEngine voice.
    void getSampleplayWaveformSnapshot(float* dst, int numPoints,
                                       double windowSeconds);

    // Configure filter parameters for a given voice. This can be
    // called from the UI thread; the audio thread will consume the
    // updated coefficients on the next callback.
    void setVoiceFilter(int index, int mode, double cutoffHz, float q);

    // SoundFont / Sampleplay integration ---------------------------------

    // Associates a SoundFont2 file path with the internal
    // SampleplaySynth instance. If FluidSynth is not available or the
    // soundfont cannot be loaded, the call is ignored and Sampleplay
    // notes will remain silent.
    void setSampleplaySoundfont(const std::string& path);

    // Triggers a single Sampleplay note using the given bank/program
    // preset, MIDI key (0-127) and normalised velocity in [0,1]. The
    // note is rendered by FluidSynth and mixed with the procedural
    // oscillators.
    void triggerSampleplayNote(int bank, int program, int midiKey,
                               float velocity01);

    // Sets a global linear gain for the Sampleplay (SoundFont)
    // output. Values are clamped to [0,1]. A value of 0.0 silences
    // all Sampleplay audio immediately (used e.g. when the
    // Sampleplay module line to the master bus is muted).
    void setSampleplayOutputGain(float gain);

private:
    juce::AudioDeviceManager deviceManager_;

    double sampleRate_{44100.0};

    struct Voice {
        std::atomic<double> frequency{0.0};
        std::atomic<float> level{0.0F};
        // Waveform shape selector: 0 = sine, 1 = saw, 2 = square,
        // 3 = noise. The audio callback interprets these indices per
        // voice to generate different oscillator shapes.
        std::atomic<int> waveform{0};
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

    // Rolling mono buffer containing only the Sampleplay (SoundFont)
    // contribution before it is mixed with oscillator voices. This
    // allows Sampleplay modules to render their own waveform on
    // connections even though they do not map to an AudioEngine
    // voice index.
    float sampleplayWaveformBuffer_[kWaveformHistorySize]{};

    // Global gain applied to the Sampleplay (SoundFont) path before it
    // is mixed with the procedural oscillators. This allows the UI to
    // implement an immediate "stop" or mute of all Sampleplay audio
    // (for example, when the Sampleplay module line to the master is
    // muted) without affecting oscillator voices.
    std::atomic<float> sampleplayOutputGain_{1.0F};
    std::atomic<int> waveformWriteIndex_{0};

    // Simple per-voice RNG state used for noise waveforms.
    std::uint32_t noiseState_[kMaxVoices]{};

    // Optional FluidSynth-based synthesiser used by Sampleplay
    // modules. Owned by the audio engine and rendered alongside the
    // procedural oscillators.
    std::unique_ptr<rectai::SampleplaySynth> sampleplaySynth_;

    // Scratch buffers used to pull stereo audio from SampleplaySynth
    // before mixing it with the oscillator voices.
    std::vector<float> sampleplayLeft_;
    std::vector<float> sampleplayRight_;

    // Set to true when the audio device initialisation succeeded but
    // reported zero output channels (e.g. "no channels"). Used by
    // the UI layer to change the table colour when audio is
    // unavailable.
    bool noOutputChannels_{false};

    // Set to true when any error is reported by
    // initialiseWithDefaultDevices during construction. This is a
    // coarse indicator that audio is not in a healthy state and is
    // used primarily for visual feedback in the UI.
    bool initError_{false};
};
