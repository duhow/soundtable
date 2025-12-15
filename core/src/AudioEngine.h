#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <juce_audio_devices/juce_audio_devices.h>
#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_dsp/juce_dsp.h>

namespace rectai {
class SampleplaySynth;
class Scene;
class AudioGraph;
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

    // Maximum number of independent connection-level waveform taps
    // that can be monitored simultaneously. This is intentionally
    // modest to keep the audio callback overhead predictable while
    // still covering typical Reactable-style scenes.
    static constexpr int kMaxConnectionTaps = 64;

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
    //
    // `getVoiceWaveformSnapshot` returns the pre-filter oscillator
    // signal for that voice, suitable for visualising generator output
    // (e.g. Osc → Filter connections).
    void getVoiceWaveformSnapshot(int voiceIndex, float* dst,
                                  int numPoints, double windowSeconds);

    // Returns a snapshot of the post-filter signal for the given
    // voice, i.e. after the per-voice StateVariableTPTFilter has been
    // applied. This is used to visualise the audio leaving modules
    // such as Filter/FX or reaching the master.
    void getVoiceFilteredWaveformSnapshot(int voiceIndex, float* dst,
                                          int numPoints,
                                          double windowSeconds);

    // Copies a downsampled snapshot of the recent mono output from
    // the Sampleplay (SoundFont) path *before* the dedicated
    // Sampleplay filter into `dst`, representing approximately
    // `windowSeconds` of audio. This is used so that Sampleplay
    // modules can display their own "original" waveform on lines
    // even though they do not occupy an AudioEngine voice.
    void getSampleplayWaveformSnapshot(float* dst, int numPoints,
                                       double windowSeconds);

    // Copies a downsampled snapshot of the recent mono output from
    // the Sampleplay path *after* the dedicated Sampleplay filter
    // into `dst`. Downstream visuals such as Filter → Master
    // radials can use this to reflect the processed SoundFont
    // signal.
    void getSampleplayFilteredWaveformSnapshot(float* dst, int numPoints,
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

    // Configures an optional filter applied to the global
    // Sampleplay (SoundFont) path before it is mixed with the
    // oscillator voices. The API mirrors the per-voice filter
    // configuration used for generators: mode 0 = bypass, 1 =
    // low-pass, 2 = band-pass, 3 = high-pass.
    void setSampleplayFilter(int mode, double cutoffHz, float q);

    // Per-connection waveform taps --------------------------------------

    // Kind of low-level signal source a connection tap is attached
    // to. For now this mirrors the high-level ConnectionVisualSource
    // abstraction used by the UI: taps can observe either the
    // pre-filter oscillator signal for a given voice, the
    // post-filter per-voice signal, or the mono Sampleplay path.
    enum class ConnectionTapSourceKind {
        kNone = 0,
        kVoicePre,
        kVoicePost,
        kSampleplay
    };

    // Clears all connection-level waveform taps and their
    // configuration. Existing history buffers are left untouched but
    // effectively ignored, as all tap kinds are reset to kNone.
    void clearAllConnectionWaveformTaps();

    // Associates a logical connection key (as used by the Scene/UI)
    // with a low-level waveform source. Repeated calls with the same
    // key update the tap configuration; new keys allocate up to
    // kMaxConnectionTaps taps. Calls with out-of-range voice indices
    // or kNone kinds are ignored.
    void configureConnectionWaveformTap(const std::string& connectionKey,
                                        ConnectionTapSourceKind kind,
                                        int voiceIndex);

    // Copies a downsampled snapshot of the recent mono waveform for a
    // given logical connection into `dst`. If no tap is registered
    // for the given key, or there is not enough history, the
    // destination buffer is cleared. The snapshot shares the same
    // time base as the global and per-voice waveform histories.
    void getConnectionWaveformSnapshot(const std::string& connectionKey,
                                       float* dst, int numPoints,
                                       double windowSeconds);

    // Audio graph integration -------------------------------------------

    // Rebuilds the internal logical audio graph from the current
    // Scene snapshot. This keeps the engine aware of modules and
    // typed connections (audio/MIDI/control) even though the DSP
    // backend is still voice-based for now.
    void rebuildAudioGraphFromScene(const rectai::Scene& scene);

    // Exposes the current logical audio graph so that higher layers
    // (e.g. MainComponent) can reason about audio edges without
    // reparsing the Scene.
    [[nodiscard]] const rectai::AudioGraph& audioGraph() const noexcept
    {
        return *audioGraph_;
    }

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
    //
    // `voicePreFilterWaveformBuffer_` stores the raw oscillator
    // output before the per-voice filter is applied so that
    // connections originating from generators (e.g. Osc → Filter)
    // can visualise the unprocessed waveform.
    //
    // `voicePostFilterWaveformBuffer_` stores the signal after the
    // per-voice filter, used to visualise the audio leaving modules
    // such as Filter/FX or reaching the master.
    float voicePreFilterWaveformBuffer_[kMaxVoices]
                                      [kWaveformHistorySize]{};
    float voicePostFilterWaveformBuffer_[kMaxVoices]
                                       [kWaveformHistorySize]{};

    // Rolling mono buffer containing only the Sampleplay (SoundFont)
    // contribution after global gain and *before* the dedicated
    // Sampleplay filter is applied. This represents the "original"
    // SoundFont signal used for connections and Sampleplay radials.
    float sampleplayWaveformBuffer_[kWaveformHistorySize]{};

    // Rolling mono buffer containing the Sampleplay path *after* the
    // dedicated Sampleplay filter so that downstream visuals (e.g.
    // Filter → Master radials) can reflect the processed signal while
    // connections Sampleplay → X still display the original waveform.
    float sampleplayFilteredWaveformBuffer_[kWaveformHistorySize]{};

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

    // Optional filter applied to the Sampleplay stereo path. This
    // allows modules such as Filter to affect SoundFont audio even
    // though it does not occupy a generator voice slot.
    std::atomic<int> sampleplayFilterMode_{0};
    std::atomic<double> sampleplayFilterCutoffHz_{0.0};
    std::atomic<float> sampleplayFilterQ_{0.7071F};
    juce::dsp::StateVariableTPTFilter<float> sampleplayFilterL_{};
    juce::dsp::StateVariableTPTFilter<float> sampleplayFilterR_{};

    // Connection-level waveform taps. Each tap is configured from
    // the UI thread via a stable connection key and attached to a
    // low-level source (voice pre/post filter or Sampleplay). The
    // audio thread updates the corresponding history buffer on every
    // callback using the shared global write index, so snapshots are
    // time-aligned with the rest of the visualisation buffers.
    struct ConnectionTap {
        std::atomic<int> kind{0};       // underlying ConnectionTapSourceKind
        std::atomic<int> voiceIndex{-1};
    };

    ConnectionTap connectionTaps_[kMaxConnectionTaps]{};
    float connectionWaveformBuffers_[kMaxConnectionTaps]
                                   [kWaveformHistorySize]{};
    std::unordered_map<std::string, int> connectionKeyToTapIndex_;
    std::atomic<int> numConnectionTaps_{0};

    // Logical audio graph snapshot owned by the engine. Built from
    // the Scene and used as the canonical representation of modules
    // and typed connections on the audio side.
    std::unique_ptr<rectai::AudioGraph> audioGraph_;

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
