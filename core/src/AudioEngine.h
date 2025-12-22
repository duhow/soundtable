#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <juce_audio_devices/juce_audio_devices.h>
#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_audio_formats/juce_audio_formats.h>
#include <juce_dsp/juce_dsp.h>

#include "core/Voices.h"

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

    // Maximum number of independent LoopModule instances that expose
    // their own waveform history for visualisation. Typical patches
    // usan 4 loops; mantenemos un margen prudente.
    static constexpr int kMaxLoopWaveforms = 16;

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

    // Configure filter parameters for a given voice. This can be
    // called from the UI thread; the audio thread will consume the
    // updated coefficients on the next callback.
    void setVoiceFilter(int index, int mode, double cutoffHz, float q);

    // Explicitly retriggers the amplitude envelope for a given
    // voice. This is used by Sequencer-driven Oscillator chains so
    // that each active step can start a fresh envelope cycle even
    // when the underlying voice level does not toggle between 0 and
    // a non-zero value.
    void triggerVoiceEnvelope(int index);

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

    // Per-voice amplitude envelope configuration. Times are
    // specified in milliseconds as loaded from the Reactable .rtp
    // envelopes associated with Oscillator modules. `sustainLevel`
    // is a normalised amplitude in [0,1] typically derived from the
    // envelope's `points_y` control points.
    void setVoiceEnvelope(int index, float attackMs, float decayMs,
                          float durationMs, float releaseMs,
                          float sustainLevel);

    // Copies a downsampled snapshot of the recent mono waveform for
    // the global Sampleplay bus (mono pre-filter path) into `dst`.
    // This is backed by the dedicated visual voice representing the
    // Sampleplay mono signal so that Sampleplay-related visuals rely
    // on the unified Voices container instead of per-connection taps.
    void getSampleplayBusWaveformSnapshot(float* dst, int numPoints,
                                          double windowSeconds);

    // Copies a downsampled snapshot of the recent mono waveform for
    // a specific Loop module into `dst`. The snapshot is driven by a
    // per-module voice in the unified visual Voices container, which
    // is updated on the audio thread with the dry mono loop signal
    // after applying gain/envelope. If the module has no assigned
    // visual slot or the window is too short, the destination is
    // cleared.
    void getLoopModuleWaveformSnapshot(const std::string& moduleId,
                                       float* dst, int numPoints,
                                       double windowSeconds);

    // Copies a downsampled snapshot of the recent mono waveform for
    // the summed Loop bus output (post Loop bus filter) into `dst`.
    // This is backed by the dedicated visual voice representing the
    // Loop bus and is primarily used by FX visuals when loops are
    // upstream of a Filter/FX module.
    void getLoopBusWaveformSnapshot(float* dst, int numPoints,
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

    // Loop module integration --------------------------------------------

    // Loads a loop sample for the given logical module id and slot index
    // from an absolute file path. The audio is decoded via JUCE
    // AudioFormatManager and stored as interleaved stereo float buffers.
    //
    // Supported formats depend on the compiled-in JUCE formats but are
    // expected to include at least WAV/AIFF/FLAC/Ogg/Opus on desktop.
    //
    // On success, returns true. On failure, returns false and optionally
    // writes a human-readable error message into `outError`.
    bool loadLoopSampleFromFile(const std::string& moduleId,
                                int slotIndex,
                                const std::string& absolutePath,
                                int beats,
                                std::string* outError);

    // Updates the runtime parameters for a loop module:
    //   - `selectedIndex` selects which of the loaded slots [0,3] is
    //     currently audible.
    //   - `linearGain` is a per-module gain in [0,1] applied to the
    //     loop output before mixing with the rest of the engine.
    //
    // The internal playback phase for all slots continues advancing
    // regardless of gain so that muting a connection to the master
    // does not pause the loops.
    void setLoopModuleParams(const std::string& moduleId,
                             int selectedIndex,
                             float linearGain,
                             float attackMs,
                             float decayMs,
                             float durationMs,
                             float releaseMs,
                             bool routeThroughFilter);

    // Sets the global tempo in beats per minute used to time-align
    // loop playback. Loops with a valid `beats` value will adjust
    // their playback rate so that their length corresponds to the
    // given number of beats at the current BPM. The mapping is
    // intentionally simple and pitch-shifts the audio.
    void setLoopGlobalTempo(float bpm);

    // Updates the fractional beat phase [0,1) for the loop transport
    // so that Loop modules can derive a continuous beat position when
    // combined with the integer beat counter. This is typically
    // driven from the UI timer using the same `beatPhase_` that
    // powers visual pulses.
    void setLoopBeatPhase(double beatPhase01);

    // Optional stereo filter applied to the mixed LoopModule
    // output. Parameters are derived from the Scene in the same
    // spirit as the Sampleplay filter, using Loop → Filter
    // connections. This keeps the DSP backend simple while still
    // allowing filters to shape loop audio even though the Loop
    // engine does not occupy individual generator voices.
    void setLoopFilter(int mode, double cutoffHz, float q);

    // Loop transport: global beat counter used to align the playback
    // phase of LoopModule samples when switching between slots. This
    // counter is advanced once per tempo beat from the UI thread and
    // read on the audio thread.
    void resetLoopBeatCounter(unsigned int startBeat = 0U);
    void advanceLoopBeatCounter();
    [[nodiscard]] unsigned int loopBeatCounter() const noexcept;

    // Configures the global Delay / Reverb FX bus driven by
    // DelayModule instances in the Scene. `mode` selects the
    // processing variant: 0 = bypass, 1 = feedback delay, 2 =
    // reverb. `delayNormalised` is the normalised delay parameter in
    // [0,1] as stored in the Scene (mapped to musical beat lengths on
    // the audio thread), `feedback` controls the feedback amount in
    // [0,1] for the feedback delay mode, `reverbAmount` controls the
    // overall wetness of the reverb mode in [0,1], and `wetGain`
    // applies an additional linear gain to the delayed/reverberated
    // signal before it is mixed with the dry input.
    void setDelayFxParams(int mode, float delayNormalised,
                          float feedback, float reverbAmount,
                          float wetGain);

    // Returns the current global transport position in beats as
    // driven by the audio callback using the configured BPM and
    // sample rate. This serves as the master clock for Sequencer
    // timing and beat-synchronised visuals.
    [[nodiscard]] double transportBeats() const noexcept
    {
        return transportBeatsAudio_.load(std::memory_order_relaxed);
    }

    // Enables or disables active audio processing. When disabled and
    // there is no pending output from previous envelopes or loops,
    // the audio callback will simply clear the output buffers and
    // skip all synthesis/sampling work so that CPU usage in fully
    // idle scenes remains minimal.
    void setProcessingActive(bool active) noexcept;

    // Returns the effective beat count associated with the given
    // Loop module id and slot index. This reflects the value
    // currently used by the audio engine for tempo sync, taking into
    // account any .rtp metadata, cached beat counts and automatic
    // detection performed when loading the sample. If the module or
    // slot is missing, or the sample has no valid beat metadata,
    // returns 0.
    int getLoopSampleBeats(const std::string& moduleId,
                           int slotIndex) const;

    // Internal helper shared by `setSampleplayFilter` and
    // `setLoopFilter` to configure one of the generic stereo filter
    // buses. Exposed here so it can be defined in the implementation
    // file without duplicating the filter setup logic per bus.
    void setBusFilter(int busIndex, int mode, double cutoffHz,
                      float q);

    // Exposes the internal visual Voices container so that higher
    // layers can start consuming per-module waveforms once the
    // routing and mapping are ready. For now this is only used as a
    // sink from the audio thread; the UI still relies on the legacy
    // waveform buffers.
    [[nodiscard]] rectai::Voices* visualVoices() noexcept
    {
        return &visualVoices_;
    }

    [[nodiscard]] const rectai::Voices* visualVoices() const noexcept
    {
        return &visualVoices_;
    }

private:
    juce::AudioDeviceManager deviceManager_;

    double sampleRate_{44100.0};

    // Visual voice layout:
    //   [0, kMaxVoices)                  → Oscillator voices
    //   [kMaxVoices, kMaxVoices +
    //    kMaxLoopWaveforms)             → LoopModule dry mono voices
    //   kVisualVoiceIdSampleplayRaw      → Global Sampleplay mono path
    //                                      (pre-bus-filter), shared by
    //                                      all SampleplayModule visuals.
    //   kVisualVoiceIdLoopBus            → Summed Loop bus output
    //                                      (post Loop bus filter),
    //                                      used by Filter/FX visuals
    //                                      when loops are upstream.
    static constexpr int kVisualVoiceIdSampleplayRaw =
        kMaxVoices + kMaxLoopWaveforms;
    static constexpr int kVisualVoiceIdLoopBus =
        kVisualVoiceIdSampleplayRaw + 1;
    static constexpr int kNumVisualVoices =
        kVisualVoiceIdLoopBus + 1;

    // Visual voices container used to keep per-module (and bus-level)
    // waveform history in a unified structure. The UI samples these
    // histories via lightweight wrappers that expose per-voice and
    // per-bus snapshots.
    rectai::Voices visualVoices_{kNumVisualVoices, kWaveformHistorySize};

    // Separate pre-filter visual history for generator voices. This
    // mirrors the per-voice signal after envelope/level but before the
    // per-voice filter, allowing connections such as Oscillator →
    // Filter to display the original generator waveform while
    // downstream segments (Filter → Master) use the post-filter
    // history stored in `visualVoices_`.
    rectai::Voices oscPreVoices_{kMaxVoices, kWaveformHistorySize};

    // Maximum delay time supported by the global Delay FX bus, in
    // seconds. This is intentionally conservative to keep memory
    // usage bounded while still covering typical musical delay
    // lengths (up to ~4 bars at moderate tempos).
    static constexpr double kMaxDelaySeconds = 4.0;

    // Corresponding maximum number of samples in the delay ring
    // buffers. We size the buffers for the worst-case combination of
    // sample rate and maximum delay time we expect to support (for
    // example, 48 kHz * 4 seconds).
    static constexpr int kMaxDelaySamples = 192000;

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
        // Envelope times configured from the Scene/UI. Units are
        // milliseconds as loaded from the Reactable .rtp envelope
        // attributes. The audio thread converts them to seconds and
        // sample counts on the fly.
        std::atomic<float> attackMs{0.0F};
        std::atomic<float> decayMs{0.0F};
        std::atomic<float> durationMs{0.0F};
        std::atomic<float> releaseMs{0.0F};
        // Sustain level in [0,1] used by the ADSR-style envelope
        // runtime. Typically derived from Envelope::points_y when
        // available; defaults to 1.0 (full sustain) for legacy AR
        // behaviour.
        std::atomic<float> sustainLevel{1.0F};
    };

    Voice voices_[kMaxVoices];
    std::atomic<int> numVoices_{0};

    // Phase per voice, only touched on the audio thread.
    double phases_[kMaxVoices]{};

    // Simple per-voice amplitude envelope used to apply ADSR-style
    // shaping to generator output. The envelope is driven entirely
    // on the audio thread based on transitions of the target voice
    // level (0 → >0 = note-on, >0 → 0 = note-off).
    enum class EnvelopePhase { kIdle = 0, kAttack, kDecay, kSustain, kRelease };

    EnvelopePhase voiceEnvPhase_[kMaxVoices]{};
    double voiceEnvValue_[kMaxVoices]{};        // [0,1] envelope amplitude
    double voiceEnvTimeInPhase_[kMaxVoices]{};  // seconds elapsed in phase
    double voicePrevTargetLevel_[kMaxVoices]{}; // last seen target level
    // Base level captured at note-on for each voice so that the
    // release/decay tail can fade out smoothly even when the
    // instantaneous target level later falls to zero.
    double voiceEnvBaseLevel_[kMaxVoices]{};
    // Envelope value at the moment a note-off is detected, used as
    // the starting point for the linear release segment so that
    // releases can start from the current sustain/decay level.
    double voiceEnvReleaseStart_[kMaxVoices]{};
    // Pending envelope retrigger flags per voice, set from the UI
    // thread and consumed on the audio thread. When true, the next
    // audio callback iteration will restart the envelope in the
    // attack phase regardless of level transitions.
    std::atomic<bool> voiceEnvRetrigger_[kMaxVoices]{};

    // Per-voice state-variable filters implemented using JUCE's DSP
    // module for improved stability and sound quality.
    juce::dsp::StateVariableTPTFilter<float> filters_[kMaxVoices]{};

    // Global Delay / Reverb FX bus state. A single instance is
    // applied to the final stereo mix when a DelayModule tangible is
    // active and connected to the master Output. Parameters are
    // driven from the UI thread via `setDelayFxParams` and consumed
    // atomically on the audio thread.
    struct DelayFxState {
        std::atomic<int> mode{0};           // 0=bypass,1=delay,2=reverb
        std::atomic<float> delayNormalised{0.66F};
        std::atomic<float> feedback{0.5F};
        std::atomic<float> reverbAmount{0.5F};
        std::atomic<float> wetGain{1.0F};
    };

    DelayFxState delayFx_{};

    // Simple stereo ring buffers used to implement the feedback
    // delay line for the Delay FX mode. Accessed exclusively on the
    // audio thread.
    std::vector<float> delayBufferL_;
    std::vector<float> delayBufferR_;
    int delayWriteIndex_{0};

    // Lightweight JUCE reverb instance used for the DelayModule
    // "reverb" mode when the FX bus is active.
    juce::Reverb reverb_{};

    // Global gain applied to the Sampleplay (SoundFont) path before it
    // is mixed with the procedural oscillators. This allows the UI to
    // implement an immediate "stop" or mute of all Sampleplay audio
    // (for example, when the Sampleplay module line to the master is
    // muted) without affecting oscillator voices.
    std::atomic<float> sampleplayOutputGain_{1.0F};

    // Simple per-voice RNG state used for noise waveforms.
    std::uint32_t noiseState_[kMaxVoices]{};

    // High-level processing gate driven from the UI thread. When
    // false, and once the engine has observed a full block of
    // silence, the audio callback will short-circuit and avoid
    // generating or sampling any audio while keeping the device
    // initialised.
    std::atomic<bool> processingRequested_{false};

    // Sticky flag updated from the audio thread indicating whether
    // the previous callback block produced any non-silent output.
    // This allows release tails from envelopes, loops or Sampleplay
    // notes to finish naturally even after the UI has marked the
    // scene as having no active audio, and only then lets the engine
    // enter a fully idle, low-CPU state.
    std::atomic<bool> hasPendingOutput_{false};

    // Optional FluidSynth-based synthesiser used by Sampleplay
    // modules. Owned by the audio engine and rendered alongside the
    // procedural oscillators.
    std::unique_ptr<rectai::SampleplaySynth> sampleplaySynth_;

    // Scratch buffers used to pull stereo audio from SampleplaySynth
    // before mixing it with the oscillator voices.
    std::vector<float> sampleplayLeft_;
    std::vector<float> sampleplayRight_;

    // Generic stereo filter buses used to apply processing to
    // sound-producing paths that do not map 1:1 to oscillator
    // voices (for example, the global Sampleplay path and the mixed
    // output of all LoopModule instances). Each bus holds a simple
    // StateVariableTPTFilter pair (L/R) plus the current mode,
    // cutoff and resonance. Higher-level routing code maps Filter
    // modules onto these buses so that the AudioEngine does not
    // need to know about specific module types.
    static constexpr int kBusFilterSampleplay = 0;
    static constexpr int kBusFilterLoop = 1;
    static constexpr int kNumBusFilters = 2;

    struct BusFilterState {
        std::atomic<int> mode{0};
        std::atomic<double> cutoffHz{0.0};
        std::atomic<float> q{0.7071F};
        juce::dsp::StateVariableTPTFilter<float> filterL{};
        juce::dsp::StateVariableTPTFilter<float> filterR{};
    };

    BusFilterState busFilters_[kNumBusFilters]{};

    // Per-LoopModule waveform history ---------------------------------

    // Each active Loop module that needs a dedicated waveform curve in
    // the UI is assigned a small integer slot. The audio thread writes
    // the dry mono loop signal (pre-mix gain) into the unified
    // `visualVoices_` container using a per-module voice id, and the
    // GUI thread samples snapshots via `getLoopModuleWaveformSnapshot`.
    std::unordered_map<std::string, int> loopModuleToWaveformIndex_;
    int numLoopWaveformSlots_{0};

    // Logical audio graph snapshot owned by the engine. Built from
    // the Scene and used as the canonical representation of modules
    // and typed connections on the audio side.
    std::unique_ptr<rectai::AudioGraph> audioGraph_;

    // ------------------------------------------------------------------
    // Loop modules

    struct LoopSharedBuffer {
        std::vector<float> interleavedData;  // [L,R,L,R,...]
        int numFrames{0};
        double sourceSampleRate{0.0};
    };

    struct LoopSample {
        std::shared_ptr<LoopSharedBuffer> buffer;
        int beats{0};
    };

    struct LoopInstance {
        // Per-slot decoded audio; typically up to 4 entries.
        std::vector<LoopSample> slots;
        // Shared playback position per slot, in source frames. Only
        // touched on the audio thread.
        std::vector<double> readPositions;
        // Last slot index whose playback position was aligned on the
        // audio thread. Used to detect when the selected slot changes
        // so that we can derive a new read position from the global
        // loop beat counter.
        int lastSelectedIndexForPlayback{-1};
        // Index of the currently audible slot.
        std::atomic<int> selectedIndex{0};
        // Linear gain in [0,1] applied to the selected slot.
        std::atomic<float> gain{0.0F};

        // Simple AR-style envelope configuration (milliseconds) and
        // runtime state used to smooth gain changes for Loop
        // modules. For now we use attack/release only; decay and
        // duration are kept for future refinement when modelling the
        // full ADSR from Reactable.
        std::atomic<float> attackMs{0.0F};
        std::atomic<float> decayMs{0.0F};
        std::atomic<float> durationMs{0.0F};
        std::atomic<float> releaseMs{0.0F};

        EnvelopePhase envPhase{EnvelopePhase::kIdle};
        double envValue{0.0};
        double envTimeInPhase{0.0};
        double prevTargetGain{0.0};

        // Index into the visual loop waveform slots used solely for
        // visualisation. Managed from the UI thread when
        // `setLoopModuleParams` is called and consumed on the audio
        // thread via the snapshot map.
        int visualWaveformIndex{-1};

        // When true, this Loop instance is considered upstream of the
        // global Loop bus filter. Its audio contribution is routed
        // through the Loop bus (and thus affected by the selected
        // Filter module) instead of being mixed directly into the
        // master output. Controlled from the UI thread according to
        // active Loop → Filter connections.
        std::atomic<bool> routeThroughFilter{false};

        LoopInstance() = default;

        // Custom copy constructor/assignment to allow the container of
        // LoopInstance to be copied even though std::atomic is not
        // copy-constructible by default.
        LoopInstance(const LoopInstance& other)
        {
            slots = other.slots;
            readPositions = other.readPositions;
            lastSelectedIndexForPlayback =
                other.lastSelectedIndexForPlayback;
            selectedIndex.store(
                other.selectedIndex.load(std::memory_order_relaxed),
                std::memory_order_relaxed);
            gain.store(other.gain.load(std::memory_order_relaxed),
                       std::memory_order_relaxed);
            visualWaveformIndex = other.visualWaveformIndex;
            routeThroughFilter.store(
                other.routeThroughFilter.load(
                    std::memory_order_relaxed),
                std::memory_order_relaxed);
        }

        LoopInstance& operator=(const LoopInstance& other)
        {
            if (this != &other) {
                slots = other.slots;
                readPositions = other.readPositions;
                lastSelectedIndexForPlayback =
                    other.lastSelectedIndexForPlayback;
                selectedIndex.store(
                    other.selectedIndex.load(std::memory_order_relaxed),
                    std::memory_order_relaxed);
                gain.store(
                    other.gain.load(std::memory_order_relaxed),
                    std::memory_order_relaxed);
                visualWaveformIndex = other.visualWaveformIndex;
                routeThroughFilter.store(
                    other.routeThroughFilter.load(
                        std::memory_order_relaxed),
                    std::memory_order_relaxed);
            }
            return *this;
        }

        LoopInstance(LoopInstance&& other) noexcept
        {
            slots = std::move(other.slots);
            readPositions = std::move(other.readPositions);
            lastSelectedIndexForPlayback =
                other.lastSelectedIndexForPlayback;
            selectedIndex.store(
                other.selectedIndex.load(std::memory_order_relaxed),
                std::memory_order_relaxed);
            gain.store(other.gain.load(std::memory_order_relaxed),
                       std::memory_order_relaxed);
            visualWaveformIndex = other.visualWaveformIndex;
            routeThroughFilter.store(
                other.routeThroughFilter.load(
                    std::memory_order_relaxed),
                std::memory_order_relaxed);
        }

        LoopInstance& operator=(LoopInstance&& other) noexcept
        {
            if (this != &other) {
                slots = std::move(other.slots);
                readPositions = std::move(other.readPositions);
                lastSelectedIndexForPlayback =
                    other.lastSelectedIndexForPlayback;
                selectedIndex.store(
                    other.selectedIndex.load(std::memory_order_relaxed),
                    std::memory_order_relaxed);
                gain.store(
                    other.gain.load(std::memory_order_relaxed),
                    std::memory_order_relaxed);
                visualWaveformIndex = other.visualWaveformIndex;
                routeThroughFilter.store(
                    other.routeThroughFilter.load(
                        std::memory_order_relaxed),
                    std::memory_order_relaxed);
            }
            return *this;
        }
    };

    // Mutable map updated from the UI thread when loop samples are
    // loaded or their parameters change.
    std::unordered_map<std::string, LoopInstance> loopModules_;

    // Snapshot pointer used by the audio thread for lock-free
    // iteration over loop instances.
    std::shared_ptr<std::unordered_map<std::string, LoopInstance>>
        loopModulesSnapshot_;

    // Global tempo in BPM used for approximate loop sync.
    std::atomic<float> loopGlobalBpm_{120.0F};

    // Global transport position in beats driven from the audio
    // thread. This is derived from the current BPM and sample
    // rate and exposed to the UI as a high-precision master
    // clock so that Sequencer timing and visual beat pulses can
    // stay phase-locked to the actual audio stream.
    std::atomic<double> transportBeatsAudio_{0.0};

    // Internal non-atomic accumulator for `transportBeatsAudio_`,
    // only touched on the audio thread.
    double transportBeatsInternal_{0.0};

    // Global integer beat counter for Loop modules. Incremented once
    // per tempo beat so that individual loop slots can derive a
    // consistent playback phase when switching between samples.
    std::atomic<unsigned int> loopGlobalBeatCounter_{0U};

    // Fractional beat phase in [0,1) used together with
    // `loopGlobalBeatCounter_` to obtain a continuous global beat
    // position for LoopModule playback alignment.
    std::atomic<double> loopBeatPhase_{0.0};

    // Non-copyable helper to access the AudioFormatManager without
    // reinitialising it on every load.
    juce::AudioFormatManager loopFormatManager_;

    // Cache of decoded loop buffers keyed by absolute path so that
    // multiple LoopModule instances or slots referencing the same
    // audio file can share storage instead of re-decoding the file
    // into separate buffers.
    std::unordered_map<std::string, std::shared_ptr<LoopSharedBuffer>>
        loopSampleCache_;

    // Optional cache of canonical beat counts per decoded loop
    // sample, keyed by the same absolute path used in
    // `loopSampleCache_`. When a Loop sample is first loaded from a
    // .rtp session, the explicit `beats` metadata from Reactable is
    // stored here so that subsequent selections of the same audio
    // file from the Loop File Browser can reuse that value even if
    // the caller passes `beats <= 0` to request auto-detection. When
    // no prior metadata exists, auto-detected beat counts derived
    // from the file duration and current loop BPM are also cached so
    // that repeated loads remain stable.
    std::unordered_map<std::string, int> loopSampleBeatsCache_;

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
