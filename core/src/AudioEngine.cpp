#include "AudioEngine.h"

#include <algorithm>
#include <cmath>

#include "core/AudioGraph.h"
#include "core/SampleplaySynth.h"

AudioEngine::AudioEngine()
{
    // Initialise with no inputs and stereo outputs.
    juce::String audioError =
        deviceManager_.initialiseWithDefaultDevices(/*numInputChannels*/ 0,
                                                    /*numOutputChannels*/ 2);
    if (audioError.isNotEmpty()) {
        juce::Logger::writeToLog("[rectai-core] Failed to initialise audio: " +
                                 audioError);

        // Detect the specific case where JUCE reports that there are
        // no available output channels (for example, when the system
        // has no active audio device). The UI can surface this state
        // by changing the table colour.
        initError_ = true;
        if (audioError.containsIgnoreCase("no channels")) {
            noOutputChannels_ = true;
        }
    } else {
        deviceManager_.addAudioCallback(this);
        juce::Logger::writeToLog("[rectai-core] Audio engine initialised.");
    }

    // Prepare SampleplaySynth instance; the actual SoundFont and
    // sample rate will be configured later from MainComponent and
    // audioDeviceAboutToStart respectively.
    sampleplaySynth_ = std::make_unique<rectai::SampleplaySynth>();

    // Lazily construct the logical audio graph container so that the
    // engine can keep track of modules and typed connections once a
    // Scene is provided.
    audioGraph_ = std::make_unique<rectai::AudioGraph>();

    // Register basic audio formats once; used for Loop module sample
    // decoding (WAV/FLAC/Ogg/Opus, depending on JUCE configuration).
    loopFormatManager_.registerBasicFormats();
}

AudioEngine::~AudioEngine()
{
    deviceManager_.removeAudioCallback(this);
}

void AudioEngine::audioDeviceAboutToStart(juce::AudioIODevice* device)
{
    sampleRate_ = (device != nullptr && device->getCurrentSampleRate() > 0.0)
                      ? device->getCurrentSampleRate()
                      : 44100.0;
    transportBeatsInternal_ = 0.0;
    transportBeatsAudio_.store(0.0, std::memory_order_relaxed);
    // Reset phases for all voices.
    for (double& phase : phases_) {
        phase = 0.0;
    }

    // Reset per-voice envelope runtime state so that any stale
    // ADSR phases from a previous device configuration do not leak
    // into the new session.
    for (int v = 0; v < kMaxVoices; ++v) {
        voiceEnvPhase_[v] = EnvelopePhase::kIdle;
        voiceEnvValue_[v] = 0.0;
        voiceEnvTimeInPhase_[v] = 0.0;
        voicePrevTargetLevel_[v] = 0.0;
        voiceEnvBaseLevel_[v] = 0.0;
    }

    const juce::uint32 maxBlockSize =
        (device != nullptr && device->getCurrentBufferSizeSamples() > 0)
            ? static_cast<juce::uint32>(
                  device->getCurrentBufferSizeSamples())
            : 512U;

    juce::dsp::ProcessSpec spec{sampleRate_, maxBlockSize, 1U};

    for (int v = 0; v < kMaxVoices; ++v) {
        filters_[v].reset();
        filters_[v].prepare(spec);
        voices_[v].filterMode.store(0, std::memory_order_relaxed);
        voices_[v].filterCutoffHz.store(0.0, std::memory_order_relaxed);
        voices_[v].filterQ.store(0.7071F, std::memory_order_relaxed);
        voices_[v].waveform.store(0, std::memory_order_relaxed);
        noiseState_[v] = 0x1234567u + static_cast<std::uint32_t>(v) *
                                         0x01010101u;
    }

    // Prepare Sampleplay filters (stereo, implemented as two
    // independent mono filters sharing parameters).
    sampleplayFilterL_.reset();
    sampleplayFilterR_.reset();
    sampleplayFilterL_.prepare(spec);
    sampleplayFilterR_.prepare(spec);
    sampleplayFilterMode_.store(0, std::memory_order_relaxed);
    sampleplayFilterCutoffHz_.store(0.0,
                                    std::memory_order_relaxed);
    sampleplayFilterQ_.store(0.7071F, std::memory_order_relaxed);

    // Resize Sampleplay scratch buffers to fit the maximum expected
    // block size and inform the internal synth about the current
    // sample rate.
    sampleplayLeft_.assign(static_cast<std::size_t>(maxBlockSize), 0.0F);
    sampleplayRight_.assign(static_cast<std::size_t>(maxBlockSize), 0.0F);

    if (sampleplaySynth_ != nullptr) {
        sampleplaySynth_->setSampleRate(sampleRate_);
    }
}

void AudioEngine::audioDeviceStopped()
{
    for (double& phase : phases_) {
        phase = 0.0;
    }

    for (int v = 0; v < kMaxVoices; ++v) {
        filters_[v].reset();
    }

    sampleplayFilterL_.reset();
    sampleplayFilterR_.reset();
}

void AudioEngine::audioDeviceIOCallbackWithContext(
    const float* const* /*inputChannelData*/,
    const int /*numInputChannels*/,
    float* const* outputChannelData,
    const int numOutputChannels,
    const int numSamples,
    const juce::AudioIODeviceCallbackContext& /*context*/)
{
    if (sampleRate_ <= 0.0) {
        // Clear outputs if the device is not properly started yet.
        for (int channel = 0; channel < numOutputChannels; ++channel) {
            if (auto* buffer = outputChannelData[channel]) {
                std::fill(buffer, buffer + numSamples, 0.0F);
            }
        }
        return;
    }

    // Fast-path exit when there is no active audio requested by the
    // scene and the previous callback block also produced silence.
    // This keeps the audio device initialised but avoids running the
    // heavier synthesis/sampling path while the table is completely
    // idle (for example, when no modules are inside the musical
    // area).
    const bool shouldProcess =
        processingRequested_.load(std::memory_order_relaxed);
    const bool hadPendingOutput =
        hasPendingOutput_.load(std::memory_order_relaxed);

    if (!shouldProcess && !hadPendingOutput) {
        for (int channel = 0; channel < numOutputChannels; ++channel) {
            if (auto* buffer = outputChannelData[channel]) {
                std::fill(buffer, buffer + numSamples, 0.0F);
            }
        }
        return;
    }

    const double twoPiOverFs =
        juce::MathConstants<double>::twoPi / sampleRate_;

    const int historySize = kWaveformHistorySize;
    const int voiceCount =
        std::min(numVoices_.load(std::memory_order_relaxed), kMaxVoices);

    // Number of active connection-level taps to update for this
    // callback. This value is stable for the duration of the
    // callback even if the UI thread reconfigures taps concurrently;
    // newly added taps will start receiving data on the next block.
    const int tapCount = std::min(
        numConnectionTaps_.load(std::memory_order_relaxed),
        kMaxConnectionTaps);

    // Reserve a contiguous block of indices in the circular history
    // buffer for this callback.
    const int baseWriteIndex =
        waveformWriteIndex_.fetch_add(numSamples, std::memory_order_relaxed);

    // Pull any active Sampleplay voices from the FluidSynth-backed
    // synthesiser into scratch buffers. If the buffers are smaller
    // than the current block, grow them on the fly.
    if (static_cast<int>(sampleplayLeft_.size()) < numSamples) {
        sampleplayLeft_.resize(static_cast<std::size_t>(numSamples), 0.0F);
    }
    if (static_cast<int>(sampleplayRight_.size()) < numSamples) {
        sampleplayRight_.resize(static_cast<std::size_t>(numSamples), 0.0F);
    }

    std::fill(sampleplayLeft_.begin(),
              sampleplayLeft_.begin() + numSamples, 0.0F);
    std::fill(sampleplayRight_.begin(),
              sampleplayRight_.begin() + numSamples, 0.0F);

    if (sampleplaySynth_ != nullptr) {
        sampleplaySynth_->render(sampleplayLeft_.data(),
                                 sampleplayRight_.data(), numSamples);
    }

    // Apply global Sampleplay gain so the UI can implement an
    // immediate stop/mute of all SoundFont audio (for example when
    // the Sampleplay module line to the master is muted).
    const float spGain =
        sampleplayOutputGain_.load(std::memory_order_relaxed);
    if (spGain < 0.999F) {
        const float clamped = juce::jlimit(0.0F, 1.0F, spGain);
        for (int i = 0; i < numSamples; ++i) {
            sampleplayLeft_[static_cast<std::size_t>(i)] *= clamped;
            sampleplayRight_[static_cast<std::size_t>(i)] *= clamped;
        }
    }

    // Take a snapshot of the current loop modules so that the audio
    // thread can iterate safely without holding a lock while the UI
    // thread potentially updates the map. If no snapshot exists yet
    // fall back to the mutable map directly.
    std::shared_ptr<std::unordered_map<std::string, LoopInstance>>
        loopSnapshot;
    {
        loopSnapshot = loopModulesSnapshot_;
        if (loopSnapshot == nullptr) {
            // This is only expected during startup before any loop
            // modules are configured. Using the mutable map here is
            // acceptable as it is typically empty and avoids an
            // extra allocation.
            loopSnapshot = std::make_shared<
                std::unordered_map<std::string, LoopInstance>>(
                loopModules_);
        }
    }

    const float bpm = loopGlobalBpm_.load(std::memory_order_relaxed);

    // Advance the global transport position in beats using the
    // current BPM and sample rate. This provides a high-precision
    // audio-driven clock that can be queried from the UI so that
    // Sequencer timing and visual beat pulses remain locked to the
    // actual audio stream instead of the GUI timer.
    const double beatsPerSecond =
        (bpm > 0.0F) ? (static_cast<double>(bpm) / 60.0) : 0.0;
    const double blockDurationSeconds =
        (sampleRate_ > 0.0)
            ? (static_cast<double>(numSamples) / sampleRate_)
            : 0.0;
    const double blockBeats = beatsPerSecond * blockDurationSeconds;
    transportBeatsInternal_ += blockBeats;
    transportBeatsAudio_.store(transportBeatsInternal_,
                               std::memory_order_relaxed);

    // Use the audio-driven transport position for Loop alignment so
    // that Loop playback stays in phase with the same master clock
    // used by Sequencer timing and beat pulses.
    const double loopBeats = transportBeatsInternal_;

    bool blockHasNonZeroOutput = false;

    for (int sample = 0; sample < numSamples; ++sample) {
        float oscMixed = 0.0F;

        const int writeIndex = baseWriteIndex + sample;
        const int bufIndex =
            historySize > 0 ? (writeIndex % historySize) : 0;

        // Optionally run the Sampleplay stereo path through a
        // StateVariableTPTFilter so that downstream Filter modules
        // can affect SoundFont audio when connected. We keep the
        // pre-filter mono sample (raw) and the post-filter mono
        // sample in separate history buffers so that connections
        // Sampleplay → X can display the original SoundFont
        // waveform while radials aguas abajo reflejan la señal ya
        // procesada.
        const int spFilterMode =
            sampleplayFilterMode_.load(std::memory_order_relaxed);
        const std::size_t spIdx =
            static_cast<std::size_t>(sample);
        const float rawSampleplayL =
            sampleplayLeft_[spIdx];  // after global gain
        const float rawSampleplayR =
            sampleplayRight_[spIdx];

        float filteredSampleplayL = rawSampleplayL;
        float filteredSampleplayR = rawSampleplayR;
        if (spFilterMode != 0) {
            filteredSampleplayL =
                sampleplayFilterL_.processSample(0, rawSampleplayL);
            filteredSampleplayR =
                sampleplayFilterR_.processSample(0, rawSampleplayR);
        }

        sampleplayLeft_[spIdx] = filteredSampleplayL;
        sampleplayRight_[spIdx] = filteredSampleplayR;

        // Mix oscillator voices (if any).
        if (voiceCount > 0) {
            for (int v = 0; v < voiceCount; ++v) {
                const double freq =
                    voices_[v].frequency.load(std::memory_order_relaxed);
                const float targetLevel =
                    voices_[v].level.load(std::memory_order_relaxed);

                const int waveformIndex =
                    voices_[v].waveform.load(std::memory_order_relaxed);

                float raw = 0.0F;
                // Generate waveform if frequency is set, regardless of level.
                // This allows waveform visualization even when output is muted (level=0).
                if (freq > 0.0) {
                    phases_[v] += twoPiOverFs * freq;

                    const double phase = phases_[v];
                    const double s = std::sin(phase);

                    switch (waveformIndex) {
                        case 1: { // Saw
                            const double twoPi =
                                juce::MathConstants<double>::twoPi;
                            double local = std::fmod(phase, twoPi);
                            if (local < 0.0) {
                                local += twoPi;
                            }
                            const float t = static_cast<float>(
                                local * (1.0 / twoPi));  // [0,1)
                            const float saw = 2.0F * t - 1.0F;  // [-1,1]
                            // Generate full-amplitude waveform, scale by level later for output.
                            raw = saw;
                            break;
                        }
                        case 2: { // Square
                            const float sq = (s >= 0.0 ? 1.0F : -1.0F);
                            raw = sq;
                            break;
                        }
                        case 3: { // Noise
                            std::uint32_t state = noiseState_[v];
                            // xorshift32
                            state ^= state << 13U;
                            state ^= state >> 17U;
                            state ^= state << 5U;
                            noiseState_[v] = state;
                            const float n =
                                static_cast<float>(
                                    static_cast<std::int32_t>(state)) /
                                2147483647.0F;  // ~[-1,1]
                            raw = n;
                            break;
                        }
                        case 0:
                        default: { // Sine
                            raw = static_cast<float>(s);
                            break;
                        }
                    }
                }

                // Apply optional per-voice filter via JUCE's
                // StateVariableTPTFilter for better stability.
                float s = raw;
                const int mode =
                    voices_[v].filterMode.load(std::memory_order_relaxed);
                if (mode != 0) {
                    s = filters_[v].processSample(0, raw);
                }

                // Evaluate the simple ADSR-style envelope for this
                // voice based on transitions of the target level.
                double envAmp = 1.0;
                {
                    const double lastTarget = voicePrevTargetLevel_[v];
                    const double lvl = static_cast<double>(targetLevel);
                    const bool wasOn = lastTarget > 1.0e-4;
                    const bool isOn = lvl > 1.0e-4;

                    // Consume any explicit retrigger request coming
                    // from the UI thread (e.g. Sequencer-driven note
                    // events for Oscillator modules). When a
                    // retrigger is pending we unconditionally start a
                    // new envelope cycle regardless of level
                    // transitions so que cada step pueda generar un
                    // pulso independiente.
                    const bool retrigger =
                        voiceEnvRetrigger_[v].exchange(
                            false, std::memory_order_acq_rel);

                    const bool noteOn = retrigger || (!wasOn && isOn);
                    // Note-off basado en nivel solo se usa en el modo
                    // AR clásico (sin `duration`). Para envelopes con
                    // `duration > 0` el final de la nota está
                    // gobernado únicamente por el tiempo.

                    voicePrevTargetLevel_[v] = lvl;

                    const float attackMs = voices_[v].attackMs.load(
                        std::memory_order_relaxed);
                    const float decayMs = voices_[v].decayMs.load(
                        std::memory_order_relaxed);
                    const float releaseMs = voices_[v].releaseMs.load(
                        std::memory_order_relaxed);
                    const float durationMs = voices_[v].durationMs.load(
                        std::memory_order_relaxed);
                    const float sustainLevelRaw =
                        voices_[v].sustainLevel.load(
                            std::memory_order_relaxed);

                    const double attackSecondsRaw =
                        std::max(0.0, static_cast<double>(attackMs) /
                                            1000.0);
                    const double decaySeconds =
                        std::max(0.0, static_cast<double>(decayMs) /
                                            1000.0);
                    const double releaseSeconds =
                        std::max(0.0, static_cast<double>(releaseMs) /
                                            1000.0);
                    const double durationSeconds =
                        std::max(0.0,
                                 static_cast<double>(durationMs) /
                                     1000.0);

                    // Avoid hard clicks when triggering Oscillator
                    // envelopes with attack=0 by enforcing a very
                    // small minimum attack time. This keeps the
                    // envelope effectively "instant" from a
                    // musical perspective while smoothing the
                    // first few samples enough to remove audible
                    // transients.
                    double attackSeconds = attackSecondsRaw;
                    if (attackSecondsRaw <= 0.0) {
                        constexpr double kMinAttackSeconds = 0.0005;  // 0.5 ms
                        attackSeconds = kMinAttackSeconds;
                    }

                    const bool hasOneShot = durationSeconds > 0.0;

                    // Clamp sustain level to a sensible range.
                    const double sustainLevel = juce::jlimit(
                        0.0, 1.0,
                        static_cast<double>(sustainLevelRaw));

                    if (noteOn) {
                        // Note-on: restart envelope en el inicio del
                        // ciclo y captura el nivel base para este
                        // trigger.
                        voiceEnvPhase_[v] = EnvelopePhase::kAttack;
                        voiceEnvTimeInPhase_[v] = 0.0;
                        voiceEnvValue_[v] = 0.0;
                        voiceEnvBaseLevel_[v] = lvl;
                    }

                    const double sr = sampleRate_ > 0.0 ? sampleRate_
                                                         : 44100.0;
                    const double dt = (sr > 0.0) ? (1.0 / sr) : 0.0;

                    auto phase = voiceEnvPhase_[v];
                    double value = voiceEnvValue_[v];
                    double tInPhase = voiceEnvTimeInPhase_[v];

                    if (hasOneShot) {
                        // Modo one-shot gobernado por `duration`: la
                        // envolvente recorre Attack→Decay hasta llegar
                        // a cero en `durationMs`, sin depender de un
                        // note-off explícito ni del valor actual de
                        // `level`.
                        const double totalSeconds = durationSeconds;
                        const double tailSeconds =
                            std::max(0.0, totalSeconds - attackSeconds);

                        switch (phase) {
                            case EnvelopePhase::kIdle:
                                value = 0.0;
                                break;
                            case EnvelopePhase::kAttack: {
                                if (attackSeconds <= 0.0) {
                                    value = 1.0;
                                    phase = EnvelopePhase::kDecay;
                                    tInPhase = 0.0;
                                } else {
                                    tInPhase += dt;
                                    const double norm = std::min(
                                        1.0, tInPhase / attackSeconds);
                                    value = norm;
                                    if (norm >= 1.0) {
                                        phase = EnvelopePhase::kDecay;
                                        tInPhase = 0.0;
                                    }
                                }
                                break;
                            }
                            case EnvelopePhase::kDecay:
                            case EnvelopePhase::kSustain:
                            case EnvelopePhase::kRelease: {
                                if (tailSeconds <= 0.0) {
                                    value = 0.0;
                                    phase = EnvelopePhase::kIdle;
                                    tInPhase = 0.0;
                                } else {
                                    tInPhase += dt;
                                    const double norm = std::min(
                                        1.0, tInPhase / tailSeconds);
                                    value = std::max(0.0, 1.0 - norm);
                                    if (norm >= 1.0) {
                                        value = 0.0;
                                        phase = EnvelopePhase::kIdle;
                                        tInPhase = 0.0;
                                        voiceEnvBaseLevel_[v] = 0.0;
                                    }
                                }
                                break;
                            }
                        }
                    } else {
                        // Modo ADSR clásico gobernado por
                        // transiciones del nivel objetivo cuando
                        // `duration == 0`: la envolvente recorre
                        // Attack → (opcionalmente) Decay → Sustain →
                        // Release. El nivel de sustain se deriva del
                        // envelope asociado al módulo (por ejemplo,
                        // a partir de `points_y`).
                        const bool noteOff = wasOn && !isOn && !retrigger;

                        if (noteOn) {
                            voiceEnvPhase_[v] = EnvelopePhase::kAttack;
                            voiceEnvTimeInPhase_[v] = 0.0;
                            voiceEnvValue_[v] = 0.0;
                            voiceEnvBaseLevel_[v] = lvl;
                            phase = EnvelopePhase::kAttack;
                            tInPhase = 0.0;
                            value = 0.0;
                        } else if (noteOff) {
                            voiceEnvPhase_[v] = EnvelopePhase::kRelease;
                            voiceEnvTimeInPhase_[v] = 0.0;
                            // Captura el valor actual de la
                            // envolvente para iniciar el segmento de
                            // release desde el nivel presente
                            // (típicamente el sustain), evitando
                            // saltos al pasar a Release.
                            voiceEnvReleaseStart_[v] = value;
                            phase = EnvelopePhase::kRelease;
                            tInPhase = 0.0;
                        }

                        switch (phase) {
                            case EnvelopePhase::kIdle:
                                value = 0.0;
                                break;
                            case EnvelopePhase::kAttack: {
                                if (attackSeconds <= 0.0) {
                                    value = 1.0;
                                    // Si no hay decay definido o el
                                    // sustain es 1.0, saltamos
                                    // directamente a Sustain.
                                    if (decaySeconds <= 0.0 ||
                                        sustainLevel >= 0.999) {
                                        phase = EnvelopePhase::kSustain;
                                    } else {
                                        phase = EnvelopePhase::kDecay;
                                    }
                                    tInPhase = 0.0;
                                } else {
                                    tInPhase += dt;
                                    const double norm = std::min(
                                        1.0, tInPhase / attackSeconds);
                                    value = norm;
                                    if (norm >= 1.0) {
                                        // Transición a Decay sólo si
                                        // hay un tramo de decay
                                        // definido y el sustain es
                                        // distinto de 1.0.
                                        if (decaySeconds > 0.0 &&
                                            sustainLevel < 0.999) {
                                            phase = EnvelopePhase::kDecay;
                                        } else {
                                            phase = EnvelopePhase::kSustain;
                                        }
                                        tInPhase = 0.0;
                                    }
                                }
                                break;
                            }
                            case EnvelopePhase::kDecay: {
                                if (decaySeconds <= 0.0 ||
                                    sustainLevel >= 0.999) {
                                    // Sin decay explícito o sustain
                                    // a 1.0: pasar directamente a
                                    // sustain.
                                    value = sustainLevel;
                                    phase = EnvelopePhase::kSustain;
                                    tInPhase = 0.0;
                                } else {
                                    tInPhase += dt;
                                    const double norm = std::min(
                                        1.0, tInPhase / decaySeconds);
                                    // Interpolación lineal de 1.0
                                    // hacia `sustainLevel`.
                                    value = 1.0 +
                                             (sustainLevel - 1.0) *
                                                 norm;
                                    if (norm >= 1.0) {
                                        value = sustainLevel;
                                        phase = EnvelopePhase::kSustain;
                                        tInPhase = 0.0;
                                    }
                                }
                                break;
                            }
                            case EnvelopePhase::kSustain:
                                // Mantiene el nivel de sustain
                                // mientras la nota siga activa.
                                value = sustainLevel;
                                break;
                            case EnvelopePhase::kRelease: {
                                if (releaseSeconds <= 0.0) {
                                    value = 0.0;
                                    phase = EnvelopePhase::kIdle;
                                    tInPhase = 0.0;
                                } else {
                                    tInPhase += dt;
                                    const double norm = std::min(
                                        1.0, tInPhase / releaseSeconds);
                                    const double startAmp =
                                        voiceEnvReleaseStart_[v];
                                    const double envVal = std::max(
                                        0.0,
                                        startAmp * (1.0 - norm));
                                    value = envVal;
                                    if (norm >= 1.0 || envVal <= 0.0) {
                                        value = 0.0;
                                        phase = EnvelopePhase::kIdle;
                                        tInPhase = 0.0;
                                        voiceEnvBaseLevel_[v] = 0.0;
                                    }
                                }
                                break;
                            }
                        }
                    }

                    voiceEnvPhase_[v] = phase;
                    voiceEnvValue_[v] = value;
                    voiceEnvTimeInPhase_[v] = tInPhase;
                    envAmp = value;
                }

                // Determine the effective gain for this voice by
                // combining the current target level (driven by the
                // UI volume bar and Sequencer) with the envelope. In
                // normal operation (attack/decay/sustain), the
                // output should follow the live target level so que
                // user volume changes immediately affect the sound.
                // During the release tail, if the target level has
                // been driven to (or near) zero by the Sequencer, we
                // fall back to the captured base level so that the
                // decay remains audible instead of being abruptly
                // cut.
                const double targetLevelNow = static_cast<double>(
                    voices_[v].level.load(std::memory_order_relaxed));
                const double baseLevel = voiceEnvBaseLevel_[v];
                const auto currentPhase = voiceEnvPhase_[v];
                const bool inReleaseTail =
                    (currentPhase == EnvelopePhase::kRelease);
                const double levelForMix =
                    (inReleaseTail && targetLevelNow <= 1.0e-4)
                        ? baseLevel
                        : targetLevelNow;

                // Compute per-voice audio samples before and after
                // the per-voice filter, including both envelope and
                // level. These are used both for visualisation
                // buffers and for the actual mono mix.
                const float voiceSamplePre =
                    static_cast<float>(levelForMix * envAmp) * raw;
                const float voiceSamplePost =
                    static_cast<float>(levelForMix * envAmp) * s;

                // Store both pre- and post-filter waveforms for
                // visualisation so that connections from generators
                // (Osc  Filter) can reflect the actual audio
                // leaving the Oscillator (including envelope/volume)
                // while filters/FX and master visuals can reflect the
                // processed signal.
                voicePreFilterWaveformBuffer_[v][bufIndex] =
                    voiceSamplePre;
                voicePostFilterWaveformBuffer_[v][bufIndex] =
                    voiceSamplePost;

                // Update any connection taps that monitor this
                // voice. Taps can either observe the pre-filter
                // oscillator signal or the post-filter processed
                // signal, both already scaled by the current
                // envelope and level so that visuals remain in sync
                // with the audible audio.
                for (int t = 0; t < tapCount; ++t) {
                    const int tapKind =
                        connectionTaps_[t].kind.load(
                            std::memory_order_relaxed);
                    if (tapKind == 0) {
                        continue;
                    }

                    const int tapVoice =
                        connectionTaps_[t].voiceIndex.load(
                            std::memory_order_relaxed);
                    if (tapVoice != v) {
                        continue;
                    }

                    if (tapKind == static_cast<int>(
                                      ConnectionTapSourceKind::kVoicePre)) {
                        connectionWaveformBuffers_[t][bufIndex] =
                            voiceSamplePre;
                    } else if (tapKind == static_cast<int>(
                                         ConnectionTapSourceKind::kVoicePost)) {
                        connectionWaveformBuffers_[t][bufIndex] =
                            voiceSamplePost;
                    }
                }

                // Mix the post-filter, envelope- and level-shaped
                // voice signal into the global oscillator mono bus.
                oscMixed += voiceSamplePost;
            }

            // Prevent hard digital clipping when summing several
            // voices or using high-Q filter settings on the
            // oscillator side. FluidSynth output is assumed to be
            // already well behaved; we clip after mixing both.
            oscMixed = juce::jlimit(-0.9F, 0.9F, oscMixed);

            // Update any connection taps attached to the Sampleplay
            // path using the **original** (pre-filter) mono
            // waveform so that all Sampleplay-based connections
            // share a single time-aligned source. This avoids
            // maintaining a second set of global Sampleplay
            // history buffers and keeps visualisation based solely
            // on per-connection taps.
            const float sampleplayMonoRaw = rawSampleplayL;
            for (int t = 0; t < tapCount; ++t) {
                const int tapKind =
                    connectionTaps_[t].kind.load(
                        std::memory_order_relaxed);
                if (tapKind == static_cast<int>(
                                   ConnectionTapSourceKind::kSampleplay)) {
                    connectionWaveformBuffers_[t][bufIndex] =
                        sampleplayMonoRaw;
                }
            }

            float leftOut = filteredSampleplayL + oscMixed;
            float rightOut = filteredSampleplayR + oscMixed;

            // Mix Loop modules: sum the currently selected slot for
            // each instance. Playback phase for all slots advances
            // regardless of gain so that muting connections to the
            // master does not pause the loops. When the selected slot
            // changes, its playback position is realigned from the
            // global loop beat counter so that switching samples keeps
            // them phase-locked to the transport. The per-instance
            // gain is further shaped by a simple AR envelope derived
            // from the Reactable Envelope (attack/release times).
            if (loopSnapshot && !loopSnapshot->empty()) {
                for (auto& pair : *loopSnapshot) {
                    auto& instance = pair.second;
                    const int slotIndex = instance.selectedIndex.load(
                        std::memory_order_relaxed);
                    const float loopGainTarget = instance.gain.load(
                        std::memory_order_relaxed);

                    if (slotIndex < 0 ||
                        slotIndex >= static_cast<int>(
                                         instance.slots.size())) {
                        continue;
                    }

                    auto& slot = instance.slots[static_cast<std::size_t>(
                        slotIndex)];
                    const auto& buffer = slot.buffer;
                    if (buffer == nullptr ||
                        buffer->numFrames <= 0 ||
                        buffer->sourceSampleRate <= 0.0) {
                        continue;
                    }

                    if (instance.readPositions.size() !=
                        instance.slots.size()) {
                        instance.readPositions.assign(
                            instance.slots.size(), 0.0);
                    }

                    const double sr = sampleRate_ > 0.0 ? sampleRate_
                                                         : 44100.0;
                    const double srcSr = buffer->sourceSampleRate;
                    const int totalFrames = buffer->numFrames;
                    if (sr <= 0.0 || srcSr <= 0.0 || totalFrames <= 0) {
                        continue;
                    }

                    // Align playback position for the selected slot
                    // when it changes, using the continuous global
                    // beat position (integer beats + fractional
                    // phase) so that different samples behave as if
                    // they were all running in parallel.
                    if (slot.beats > 0 && totalFrames > 0 &&
                        instance.readPositions.size() ==
                            instance.slots.size() &&
                        instance.lastSelectedIndexForPlayback !=
                            slotIndex) {
                        const unsigned int beatsPerLoop =
                            static_cast<unsigned int>(slot.beats);
                        if (beatsPerLoop > 0U) {
                            const double loopBeatsMod = std::fmod(
                                loopBeats,
                                static_cast<double>(beatsPerLoop));
                            const double fracBeat =
                                loopBeatsMod /
                                static_cast<double>(beatsPerLoop);
                            const double targetFrame =
                                fracBeat *
                                static_cast<double>(totalFrames);

                            const double clampedFrame = juce::jlimit(
                                0.0,
                                static_cast<double>(
                                    std::max(totalFrames - 1, 0)),
                                targetFrame);

                            instance.readPositions[static_cast<
                                std::size_t>(slotIndex)] =
                                clampedFrame;
                            instance.lastSelectedIndexForPlayback =
                                slotIndex;
                        }
                    }

                    // Base step from source to device sample rate.
                    double step = srcSr / sr;

                    // Optional tempo sync when both BPM and beats
                    // metadata are valid. The mapping is intentionally
                    // simple and introduces pitch shift.
                    if (bpm > 0.0 && slot.beats > 0) {
                        const double loopSeconds =
                            static_cast<double>(buffer->numFrames) /
                            buffer->sourceSampleRate;
                        if (loopSeconds > 0.0) {
                            const double desiredSeconds =
                                (60.0 * static_cast<double>(slot.beats)) /
                                bpm;
                            if (desiredSeconds > 0.0) {
                                const double rate =
                                    loopSeconds / desiredSeconds;
                                step *= rate;
                            }
                        }
                    }

                    double& pos = instance.readPositions[static_cast<
                        std::size_t>(slotIndex)];
                    if (pos < 0.0) {
                        pos = 0.0;
                    }

                    const double framePos = pos;
                    const int i0 = static_cast<int>(framePos);
                    const int i1 = (i0 + 1) % totalFrames;
                    const float frac = static_cast<float>(framePos -
                                                          static_cast<
                                                              double>(
                                                              i0));

                    const std::size_t base0 =
                        static_cast<std::size_t>(i0) * 2U;
                    const std::size_t base1 =
                        static_cast<std::size_t>(i1) * 2U;
                    if (base1 + 1 >=
                        buffer->interleavedData.size()) {
                        // Defensive: malformed buffer.
                        pos = std::fmod(pos + step,
                                        static_cast<double>(
                                            std::max(totalFrames, 1)));
                    } else {
                        const float l0 =
                            buffer->interleavedData[base0 + 0];
                        const float r0 =
                            buffer->interleavedData[base0 + 1];
                        const float l1 =
                            buffer->interleavedData[base1 + 0];
                        const float r1 =
                            buffer->interleavedData[base1 + 1];

                        const float l = l0 + (l1 - l0) * frac;
                        const float r = r0 + (r1 - r0) * frac;

                        // Evaluate AR envelope for this Loop
                        // instance based on transitions of the
                        // target gain.
                        double envAmp = 1.0;
                        {
                            const double lastTarget =
                                instance.prevTargetGain;
                            const double tgt = static_cast<double>(
                                loopGainTarget);
                            const bool wasOn = lastTarget > 1.0e-4;
                            const bool isOn = tgt > 1.0e-4;

                            if (!wasOn && isOn) {
                                instance.envPhase =
                                    EnvelopePhase::kAttack;
                                instance.envTimeInPhase = 0.0;
                                instance.envValue = 0.0;
                            } else if (wasOn && !isOn) {
                                instance.envPhase =
                                    EnvelopePhase::kRelease;
                                instance.envTimeInPhase = 0.0;
                            }

                            instance.prevTargetGain = tgt;

                            const double sr =
                                sampleRate_ > 0.0 ? sampleRate_
                                                  : 44100.0;
                            const double dt =
                                (sr > 0.0) ? (1.0 / sr) : 0.0;

                            auto phase = instance.envPhase;
                            double value = instance.envValue;
                            double tInPhase =
                                instance.envTimeInPhase;

                            const float attackMs =
                                instance.attackMs.load(
                                    std::memory_order_relaxed);
                            const float releaseMs =
                                instance.releaseMs.load(
                                    std::memory_order_relaxed);

                            const double attackSeconds =
                                std::max(
                                    0.0,
                                    static_cast<double>(attackMs) /
                                        1000.0);
                            const double releaseSeconds =
                                std::max(
                                    0.0,
                                    static_cast<double>(releaseMs) /
                                        1000.0);

                            switch (phase) {
                                case EnvelopePhase::kIdle:
                                    value = 0.0;
                                    break;
                                case EnvelopePhase::kAttack: {
                                    if (attackSeconds <= 0.0) {
                                        value = 1.0;
                                        phase = EnvelopePhase::kSustain;
                                        tInPhase = 0.0;
                                    } else {
                                        tInPhase += dt;
                                        const double norm =
                                            std::min(1.0,
                                                     tInPhase /
                                                         attackSeconds);
                                        value = norm;
                                        if (norm >= 1.0) {
                                            phase =
                                                EnvelopePhase::kSustain;
                                            tInPhase = 0.0;
                                        }
                                    }
                                    break;
                                }
                                case EnvelopePhase::kDecay:
                                    // Reserved for future full ADSR
                                    // mapping.
                                    phase = EnvelopePhase::kSustain;
                                    value = 1.0;
                                    tInPhase = 0.0;
                                    break;
                                case EnvelopePhase::kSustain:
                                    value = 1.0;
                                    break;
                                case EnvelopePhase::kRelease: {
                                    if (releaseSeconds <= 0.0) {
                                        value = 0.0;
                                        phase = EnvelopePhase::kIdle;
                                        tInPhase = 0.0;
                                    } else {
                                        tInPhase += dt;
                                        const double norm =
                                            std::min(1.0,
                                                     tInPhase /
                                                         releaseSeconds);
                                        value = std::max(0.0,
                                                         1.0 - norm);
                                        if (norm >= 1.0) {
                                            phase =
                                                EnvelopePhase::kIdle;
                                            tInPhase = 0.0;
                                        }
                                    }
                                    break;
                                }
                            }

                            instance.envPhase = phase;
                            instance.envValue = value;
                            instance.envTimeInPhase = tInPhase;
                            envAmp = value;
                        }

                        const double tgt = static_cast<double>(
                            loopGainTarget);
                        const float finalGain =
                            juce::jlimit(0.0F, 1.0F,
                                         static_cast<float>(tgt *
                                                            envAmp));
                        const float monoDry = l;
                        const float monoWithGain = monoDry * finalGain;
                        if (finalGain > 0.0F) {
                            leftOut += monoWithGain;
                            rightOut += r * finalGain;
                        }

                        const int wfIndex = instance.visualWaveformIndex;
                        if (wfIndex >= 0 &&
                            wfIndex < kMaxLoopWaveforms) {
                            // Store the dry mono loop signal for
                            // visualisation so that UI effects such
                            // as mute-on-hold can silence the audio
                            // path without flattening the waveform
                            // history.
                            loopWaveformBuffers_[wfIndex][bufIndex] =
                                monoDry;
                        }

                        pos += step;
                        if (pos >= static_cast<double>(totalFrames)) {
                            pos = std::fmod(pos,
                                            static_cast<double>(
                                                std::max(
                                                    totalFrames,
                                                    1)));
                        }
                    }
                }
            }

            leftOut = juce::jlimit(-0.9F, 0.9F, leftOut);
            rightOut = juce::jlimit(-0.9F, 0.9F, rightOut);

            // Store the mixed mono signal (left channel) for
            // waveform visualisation.
            waveformBuffer_[bufIndex] = leftOut;

            if (!blockHasNonZeroOutput &&
                (leftOut != 0.0F || rightOut != 0.0F)) {
                blockHasNonZeroOutput = true;
            }

            for (int channel = 0; channel < numOutputChannels; ++channel) {
                if (auto* buffer = outputChannelData[channel]) {
                    if (channel == 0) {
                        buffer[sample] = leftOut;
                    } else if (channel == 1) {
                        buffer[sample] = rightOut;
                    } else {
                        // Duplicate left channel into any extra
                        // outputs.
                        buffer[sample] = leftOut;
                    }
                }
            }
        } else {
            // No oscillator voices are active. Still propagate
            // Sampleplay (SoundFont) audio and keep its waveform
            // history up to date so that Sampleplay-only scenes
            // both sound and render correctly.
            float leftOut =
                sampleplayLeft_[static_cast<std::size_t>(sample)];
            float rightOut =
                sampleplayRight_[static_cast<std::size_t>(sample)];
            const float sampleplayMonoRaw = rawSampleplayL;

            // No oscillator voices are active: clear per-voice
            // histories so that visualisation for generator-based
            // modules decays gracefully.
            for (int v = 0; v < kMaxVoices; ++v) {
                voicePreFilterWaveformBuffer_[v][bufIndex] = 0.0F;
                voicePostFilterWaveformBuffer_[v][bufIndex] = 0.0F;
            }

            // Update any connection taps attached to the Sampleplay
            // path usando la señal original (pre-filter).
            for (int t = 0; t < tapCount; ++t) {
                const int tapKind =
                    connectionTaps_[t].kind.load(
                        std::memory_order_relaxed);
                if (tapKind == static_cast<int>(
                                   ConnectionTapSourceKind::kSampleplay)) {
                    connectionWaveformBuffers_[t][bufIndex] =
                        sampleplayMonoRaw;
                }
            }

            // Mix Loop modules even when there are no oscillator
            // voices so that scenes consisting only of LoopModule
            // still render correctly. As in the oscillator branch,
            // align the selected slot's playback position from the
            // global loop beat counter when switching samples so that
            // different loops remain phase-locked to the transport.
            if (loopSnapshot && !loopSnapshot->empty()) {
                for (auto& pair : *loopSnapshot) {
                    auto& instance = pair.second;
                    const int slotIndex = instance.selectedIndex.load(
                        std::memory_order_relaxed);
                    const float loopGainTarget = instance.gain.load(
                        std::memory_order_relaxed);

                    if (slotIndex < 0 ||
                        slotIndex >= static_cast<int>(
                                         instance.slots.size())) {
                        continue;
                    }

                    auto& slot = instance.slots[static_cast<std::size_t>(
                        slotIndex)];
                    const auto& buffer = slot.buffer;
                    if (buffer == nullptr ||
                        buffer->numFrames <= 0 ||
                        buffer->sourceSampleRate <= 0.0) {
                        continue;
                    }

                    if (instance.readPositions.size() !=
                        instance.slots.size()) {
                        instance.readPositions.assign(
                            instance.slots.size(), 0.0);
                    }

                    const double sr = sampleRate_ > 0.0 ? sampleRate_
                                                         : 44100.0;
                    const double srcSr = buffer->sourceSampleRate;
                    const int totalFrames = buffer->numFrames;
                    if (sr <= 0.0 || srcSr <= 0.0 || totalFrames <= 0) {
                        continue;
                    }

                    // Align playback position for the selected slot
                    // when it changes, using the continuous global
                    // beat position.
                    if (slot.beats > 0 && totalFrames > 0 &&
                        instance.readPositions.size() ==
                            instance.slots.size() &&
                        instance.lastSelectedIndexForPlayback !=
                            slotIndex) {
                        const unsigned int beatsPerLoop =
                            static_cast<unsigned int>(slot.beats);
                        if (beatsPerLoop > 0U) {
                            const double loopBeatsMod = std::fmod(
                                loopBeats,
                                static_cast<double>(beatsPerLoop));
                            const double fracBeat =
                                loopBeatsMod /
                                static_cast<double>(beatsPerLoop);
                            const double targetFrame =
                                fracBeat *
                                static_cast<double>(totalFrames);

                            const double clampedFrame = juce::jlimit(
                                0.0,
                                static_cast<double>(
                                    std::max(totalFrames - 1, 0)),
                                targetFrame);

                            instance.readPositions[static_cast<
                                std::size_t>(slotIndex)] =
                                clampedFrame;
                            instance.lastSelectedIndexForPlayback =
                                slotIndex;
                        }
                    }

                    double step = srcSr / sr;
                    if (bpm > 0.0 && slot.beats > 0) {
                        const double loopSeconds =
                            static_cast<double>(buffer->numFrames) /
                            buffer->sourceSampleRate;
                        if (loopSeconds > 0.0) {
                            const double desiredSeconds =
                                (60.0 * static_cast<double>(slot.beats)) /
                                bpm;
                            if (desiredSeconds > 0.0) {
                                const double rate =
                                    loopSeconds / desiredSeconds;
                                step *= rate;
                            }
                        }
                    }

                    double& pos = instance.readPositions[static_cast<
                        std::size_t>(slotIndex)];
                    if (pos < 0.0) {
                        pos = 0.0;
                    }

                    const double framePos = pos;
                    const int i0 = static_cast<int>(framePos);
                    const int i1 = (i0 + 1) % totalFrames;
                    const float frac = static_cast<float>(framePos -
                                                          static_cast<
                                                              double>(
                                                              i0));

                    const std::size_t base0 =
                        static_cast<std::size_t>(i0) * 2U;
                    const std::size_t base1 =
                        static_cast<std::size_t>(i1) * 2U;
                    if (base1 + 1 >=
                        buffer->interleavedData.size()) {
                        pos = std::fmod(pos + step,
                                        static_cast<double>(
                                            std::max(totalFrames, 1)));
                    } else {
                        const float l0 =
                            buffer->interleavedData[base0 + 0];
                        const float r0 =
                            buffer->interleavedData[base0 + 1];
                        const float l1 =
                            buffer->interleavedData[base1 + 0];
                        const float r1 =
                            buffer->interleavedData[base1 + 1];

                        const float l = l0 + (l1 - l0) * frac;
                        const float r = r0 + (r1 - r0) * frac;

                        // Apply the same AR-style envelope used in
                        // the oscillator-voices branch so that Loop
                        // gains are smoothed consistently even when
                        // no generator voices are active.
                        double envAmp = 1.0;
                        {
                            const double lastTarget =
                                instance.prevTargetGain;
                            const double tgt = static_cast<double>(
                                loopGainTarget);
                            const bool wasOn = lastTarget > 1.0e-4;
                            const bool isOn = tgt > 1.0e-4;

                            if (!wasOn && isOn) {
                                instance.envPhase =
                                    EnvelopePhase::kAttack;
                                instance.envTimeInPhase = 0.0;
                                instance.envValue = 0.0;
                            } else if (wasOn && !isOn) {
                                instance.envPhase =
                                    EnvelopePhase::kRelease;
                                instance.envTimeInPhase = 0.0;
                            }

                            instance.prevTargetGain = tgt;

                            const double sr =
                                sampleRate_ > 0.0 ? sampleRate_
                                                  : 44100.0;
                            const double dt =
                                (sr > 0.0) ? (1.0 / sr) : 0.0;

                            auto phase = instance.envPhase;
                            double value = instance.envValue;
                            double tInPhase =
                                instance.envTimeInPhase;

                            const float attackMs =
                                instance.attackMs.load(
                                    std::memory_order_relaxed);
                            const float releaseMs =
                                instance.releaseMs.load(
                                    std::memory_order_relaxed);

                            const double attackSeconds =
                                std::max(
                                    0.0,
                                    static_cast<double>(attackMs) /
                                        1000.0);
                            const double releaseSeconds =
                                std::max(
                                    0.0,
                                    static_cast<double>(releaseMs) /
                                        1000.0);

                            switch (phase) {
                                case EnvelopePhase::kIdle:
                                    value = 0.0;
                                    break;
                                case EnvelopePhase::kAttack: {
                                    if (attackSeconds <= 0.0) {
                                        value = 1.0;
                                        phase = EnvelopePhase::kSustain;
                                        tInPhase = 0.0;
                                    } else {
                                        tInPhase += dt;
                                        const double norm =
                                            std::min(1.0,
                                                     tInPhase /
                                                         attackSeconds);
                                        value = norm;
                                        if (norm >= 1.0) {
                                            phase =
                                                EnvelopePhase::kSustain;
                                            tInPhase = 0.0;
                                        }
                                    }
                                    break;
                                }
                                case EnvelopePhase::kDecay:
                                    phase = EnvelopePhase::kSustain;
                                    value = 1.0;
                                    tInPhase = 0.0;
                                    break;
                                case EnvelopePhase::kSustain:
                                    value = 1.0;
                                    break;
                                case EnvelopePhase::kRelease: {
                                    if (releaseSeconds <= 0.0) {
                                        value = 0.0;
                                        phase = EnvelopePhase::kIdle;
                                        tInPhase = 0.0;
                                    } else {
                                        tInPhase += dt;
                                        const double norm =
                                            std::min(1.0,
                                                     tInPhase /
                                                         releaseSeconds);
                                        value = std::max(0.0,
                                                         1.0 - norm);
                                        if (norm >= 1.0) {
                                            phase =
                                                EnvelopePhase::kIdle;
                                            tInPhase = 0.0;
                                        }
                                    }
                                    break;
                                }
                            }

                            instance.envPhase = phase;
                            instance.envValue = value;
                            instance.envTimeInPhase = tInPhase;
                            envAmp = value;
                        }

                        const double tgt = static_cast<double>(
                            loopGainTarget);
                        const float finalGain =
                            juce::jlimit(0.0F, 1.0F,
                                         static_cast<float>(tgt *
                                                            envAmp));
                        const float monoDry = l;
                        const float monoWithGain = monoDry * finalGain;
                        if (finalGain > 0.0F) {
                            leftOut += monoWithGain;
                            rightOut += r * finalGain;
                        }

                        const int wfIndex = instance.visualWaveformIndex;
                        if (wfIndex >= 0 &&
                            wfIndex < kMaxLoopWaveforms) {
                            // Store the dry mono loop signal for
                            // visualisation so that UI effects such
                            // as mute-on-hold can silence the audio
                            // path without flattening the waveform
                            // history.
                            loopWaveformBuffers_[wfIndex][bufIndex] =
                                monoDry;
                        }

                        pos += step;
                        if (pos >= static_cast<double>(totalFrames)) {
                            pos = std::fmod(pos,
                                            static_cast<double>(
                                                std::max(
                                                    totalFrames,
                                                    1)));
                        }
                    }
                }
            }

            leftOut = juce::jlimit(-0.9F, 0.9F, leftOut);
            rightOut = juce::jlimit(-0.9F, 0.9F, rightOut);

            // Store the mixed mono signal (left channel) for
            // waveform visualisation so that the global
            // waveform history reflects both Sampleplay and
            // Loop-only scenes.
            waveformBuffer_[bufIndex] = leftOut;

            if (!blockHasNonZeroOutput &&
                (leftOut != 0.0F || rightOut != 0.0F)) {
                blockHasNonZeroOutput = true;
            }

            for (int channel = 0; channel < numOutputChannels; ++channel) {
                if (auto* buffer = outputChannelData[channel]) {
                    if (channel == 0) {
                        buffer[sample] = leftOut;
                    } else if (channel == 1) {
                        buffer[sample] = rightOut;
                    } else {
                        buffer[sample] = leftOut;
                    }
                }
            }
        }
    }

    // Remember whether this callback block produced any audible
    // output so that the next block can safely enter the fully idle
    // path once both the scene and the engine are silent.
    hasPendingOutput_.store(blockHasNonZeroOutput,
                            std::memory_order_relaxed);
}

void AudioEngine::triggerVoiceEnvelope(const int index)
{
    if (index < 0 || index >= kMaxVoices) {
        return;
    }

    voiceEnvRetrigger_[index].store(true, std::memory_order_release);
}

bool AudioEngine::loadLoopSampleFromFile(const std::string& moduleId,
                                         const int slotIndex,
                                         const std::string& absolutePath,
                                         const int beats,
                                         std::string* outError)
{
    if (slotIndex < 0) {
        if (outError != nullptr) {
            *outError = "Invalid slot index";
        }
        return false;
    }

    juce::File file{juce::String(absolutePath)};
    if (!file.existsAsFile()) {
        if (outError != nullptr) {
            *outError = "File does not exist";
        }
        return false;
    }
    // Normalise the path used as cache key so that repeated calls
    // for the same physical file (even across modules/slots)
    // resolve to a single decoded buffer.
    const std::string canonicalPath =
        file.getFullPathName().toStdString();

    std::shared_ptr<LoopSharedBuffer> sharedBuffer;
    {
        const auto cacheIt = loopSampleCache_.find(canonicalPath);
        if (cacheIt != loopSampleCache_.end()) {
            sharedBuffer = cacheIt->second;
        }
    }

    if (sharedBuffer == nullptr) {
        std::unique_ptr<juce::AudioFormatReader> reader(
            loopFormatManager_.createReaderFor(file));
        if (reader == nullptr) {
            if (outError != nullptr) {
                *outError = "Unsupported audio format";
            }
            return false;
        }

        const juce::int64 numSamples64 = reader->lengthInSamples;
        if (numSamples64 <= 0) {
            if (outError != nullptr) {
                *outError = "Empty audio file";
            }
            return false;
        }

        const int numFrames = static_cast<int>(
            std::min<juce::int64>(numSamples64,
                                  std::numeric_limits<int>::max()));

        const int channels = static_cast<int>(reader->numChannels);
        const double sourceRate = reader->sampleRate;
        if (sourceRate <= 0.0) {
            if (outError != nullptr) {
                *outError = "Invalid sample rate";
            }
            return false;
        }

        // We always decode to stereo interleaved float data. Mono
        // files are duplicated on both channels; multi-channel
        // files use the first two channels only.
        std::vector<float> interleaved;
        interleaved.resize(static_cast<std::size_t>(numFrames * 2),
                           0.0F);

        juce::AudioBuffer<float> tempBuffer(
            std::max(2, channels), numFrames);
        tempBuffer.clear();

        if (!reader->read(&tempBuffer, 0, numFrames, 0, true,
                          true)) {
            if (outError != nullptr) {
                *outError = "Failed to read audio data";
            }
            return false;
        }

        const float* ch0 = tempBuffer.getReadPointer(0);
        const float* ch1 = tempBuffer.getNumChannels() > 1
                                ? tempBuffer.getReadPointer(1)
                                : nullptr;

        for (int i = 0; i < numFrames; ++i) {
            const float l = ch0[i];
            const float r = ch1 != nullptr ? ch1[i] : l;
            const std::size_t base =
                static_cast<std::size_t>(i) * 2U;
            interleaved[base + 0] = l;
            interleaved[base + 1] = r;
        }

        sharedBuffer = std::make_shared<LoopSharedBuffer>();
        sharedBuffer->interleavedData = std::move(interleaved);
        sharedBuffer->numFrames = numFrames;
        sharedBuffer->sourceSampleRate = sourceRate;

        loopSampleCache_[canonicalPath] = sharedBuffer;
    }

    LoopSample sample;
    sample.buffer = std::move(sharedBuffer);
    sample.beats = std::max(0, beats);

    LoopInstance& instance = loopModules_[moduleId];
    if (static_cast<int>(instance.slots.size()) <= slotIndex) {
        instance.slots.resize(
            static_cast<std::size_t>(slotIndex + 1));
        instance.readPositions.resize(instance.slots.size(),
                                      0.0);
    }

    instance.slots[static_cast<std::size_t>(slotIndex)] =
        std::move(sample);

    // Refresh the snapshot used by the audio thread.
    loopModulesSnapshot_ =
        std::make_shared<std::unordered_map<std::string, LoopInstance>>(
            loopModules_);

    if (outError != nullptr) {
        outError->clear();
    }
    return true;
}

void AudioEngine::setLoopModuleParams(const std::string& moduleId,
                                      const int selectedIndex,
                                      const float linearGain,
                                      const float attackMs,
                                      const float decayMs,
                                      const float durationMs,
                                      const float releaseMs)
{
    auto it = loopModules_.find(moduleId);
    if (it == loopModules_.end()) {
        // Creating the instance lazily allows MainComponent to drive
        // parameters even before samples are loaded.
        it = loopModules_.emplace(moduleId, LoopInstance{}).first;
    }

    LoopInstance& instance = it->second;

    // Assign a visual waveform slot for this Loop module on first
    // use so that the audio callback can write into a dedicated
    // history buffer without realizar lookups costosos por módulo en
    // cada muestra.
    if (instance.visualWaveformIndex < 0) {
        const auto idxIt = loopModuleToWaveformIndex_.find(moduleId);
        if (idxIt != loopModuleToWaveformIndex_.end()) {
            instance.visualWaveformIndex = idxIt->second;
        } else if (numLoopWaveformSlots_ < kMaxLoopWaveforms) {
            const int newIndex = numLoopWaveformSlots_++;
            loopModuleToWaveformIndex_.emplace(moduleId, newIndex);
            instance.visualWaveformIndex = newIndex;
        }
    }
    const int clampedIndex =
        std::max(0, std::min(selectedIndex,
                             static_cast<int>(instance.slots.size()) - 1));
    instance.selectedIndex.store(clampedIndex,
                                 std::memory_order_relaxed);
    const float clampedGain = juce::jlimit(0.0F, 1.0F, linearGain);
    instance.gain.store(clampedGain, std::memory_order_relaxed);

    instance.attackMs.store(attackMs, std::memory_order_relaxed);
    instance.decayMs.store(decayMs, std::memory_order_relaxed);
    instance.durationMs.store(durationMs, std::memory_order_relaxed);
    instance.releaseMs.store(releaseMs, std::memory_order_relaxed);

    // Updating parameters does not reset playback positions so that
    // loops continue running even when muted. To honour this, we avoid
    // rebuilding the snapshot on every parameter update (which would
    // recreate LoopInstance copies with fresh readPositions). Instead,
    // the audio thread keeps using the existing snapshot and we update
    // the atomic fields used for selection and gain in-place when 
    // possible.

    if (auto snapshot = loopModulesSnapshot_) {
        auto snapIt = snapshot->find(moduleId);
        if (snapIt != snapshot->end()) {
            snapIt->second.selectedIndex.store(
                clampedIndex, std::memory_order_relaxed);
            snapIt->second.gain.store(
                clampedGain, std::memory_order_relaxed);
            snapIt->second.attackMs.store(attackMs,
                                          std::memory_order_relaxed);
            snapIt->second.decayMs.store(decayMs,
                                         std::memory_order_relaxed);
            snapIt->second.durationMs.store(durationMs,
                                            std::memory_order_relaxed);
            snapIt->second.releaseMs.store(releaseMs,
                                           std::memory_order_relaxed);
            snapIt->second.visualWaveformIndex =
                instance.visualWaveformIndex;
        }
    }
}

void AudioEngine::setLoopGlobalTempo(const float bpm)
{
    const float clamped = bpm > 0.0F ? bpm : 0.0F;
    loopGlobalBpm_.store(clamped, std::memory_order_relaxed);
}

void AudioEngine::setLoopBeatPhase(const double beatPhase01)
{
    const double clamped = juce::jlimit(0.0, 1.0, beatPhase01);
    loopBeatPhase_.store(clamped, std::memory_order_relaxed);
}

void AudioEngine::resetLoopBeatCounter(const unsigned int startBeat)
{
    loopGlobalBeatCounter_.store(startBeat, std::memory_order_relaxed);
}

void AudioEngine::advanceLoopBeatCounter()
{
    loopGlobalBeatCounter_.fetch_add(1U, std::memory_order_relaxed);
}

unsigned int AudioEngine::loopBeatCounter() const noexcept
{
    return loopGlobalBeatCounter_.load(std::memory_order_relaxed);
}

void AudioEngine::setProcessingActive(const bool active) noexcept
{
    processingRequested_.store(active, std::memory_order_relaxed);
}

void AudioEngine::setFrequency(const double frequency)
{
    setVoice(0, frequency,
             voices_[0].level.load(std::memory_order_relaxed));
}

void AudioEngine::setLevel(const float level)
{
    setVoice(0, voices_[0].frequency.load(std::memory_order_relaxed), level);
}

void AudioEngine::setVoice(const int index,
                           const double frequency,
                           const float level)
{
    if (index < 0 || index >= kMaxVoices) {
        return;
    }

    voices_[index].frequency.store(frequency, std::memory_order_relaxed);
    voices_[index].level.store(level, std::memory_order_relaxed);

    int current = numVoices_.load(std::memory_order_relaxed);
    if (index + 1 > current) {
        numVoices_.store(index + 1, std::memory_order_relaxed);
    }
}

void AudioEngine::setVoiceWaveform(const int index,
                                   const int waveformIndex)
{
    if (index < 0 || index >= kMaxVoices) {
        return;
    }

    const int clamped = juce::jlimit(0, 3, waveformIndex);
    voices_[index].waveform.store(clamped, std::memory_order_relaxed);
}

void AudioEngine::setSampleplaySoundfont(const std::string& path)
{
    if (sampleplaySynth_ == nullptr) {
        return;
    }

    std::string error;
    if (!sampleplaySynth_->loadSoundfont(path, &error)) {
        juce::Logger::writeToLog(
            juce::String("[rectai-core] Sampleplay: failed to load "
                         "soundfont in AudioEngine: ") +
            path + " (" + error + ")");
    }
}

void AudioEngine::triggerSampleplayNote(const int bank, const int program,
                                        const int midiKey,
                                        const float velocity01)
{
    if (sampleplaySynth_ == nullptr) {
        return;
    }

    sampleplaySynth_->noteOn(bank, program, midiKey, velocity01);
}

void AudioEngine::setSampleplayOutputGain(const float gain)
{
    const float clamped = juce::jlimit(0.0F, 1.0F, gain);
    sampleplayOutputGain_.store(clamped, std::memory_order_relaxed);
}

void AudioEngine::setSampleplayFilter(const int mode,
                                      const double cutoffHz,
                                      const float q)
{
    const int clampedMode = juce::jlimit(0, 3, mode);

    // Resonance must be strictly > 0 for the JUCE filter; clamp to
    // a musically useful range as with per-voice filters.
    const float qClamped = juce::jlimit(0.1F, 15.0F, q);

    const double sr = sampleRate_ > 0.0 ? sampleRate_ : 44100.0;
    const double nyquist = 0.5 * sr;
    double fc = cutoffHz;
    if (fc <= 0.0) {
        fc = 0.0;
    }
    if (fc >= nyquist) {
        fc = nyquist * 0.99;
    }

    sampleplayFilterMode_.store(clampedMode,
                                std::memory_order_relaxed);
    sampleplayFilterCutoffHz_.store(fc,
                                    std::memory_order_relaxed);
    sampleplayFilterQ_.store(qClamped, std::memory_order_relaxed);

    // If the filter is disabled or has an invalid cutoff, leave it
    // effectively bypassed. We avoid resetting here to preserve
    // continuity when parameters are toggled.
    if (clampedMode == 0 || fc <= 0.0) {
        return;
    }

    using FilterType = juce::dsp::StateVariableTPTFilterType;

    switch (clampedMode) {
        case 1:  // Low-pass
            sampleplayFilterL_.setType(FilterType::lowpass);
            sampleplayFilterR_.setType(FilterType::lowpass);
            break;
        case 2:  // Band-pass
            sampleplayFilterL_.setType(FilterType::bandpass);
            sampleplayFilterR_.setType(FilterType::bandpass);
            break;
        case 3:  // High-pass
            sampleplayFilterL_.setType(FilterType::highpass);
            sampleplayFilterR_.setType(FilterType::highpass);
            break;
        default:
            return;
    }

    sampleplayFilterL_.setCutoffFrequency(static_cast<float>(fc));
    sampleplayFilterR_.setCutoffFrequency(static_cast<float>(fc));
    sampleplayFilterL_.setResonance(qClamped);
    sampleplayFilterR_.setResonance(qClamped);
}

void AudioEngine::setVoiceEnvelope(const int index,
                                   const float attackMs,
                                   const float decayMs,
                                   const float durationMs,
                                   const float releaseMs,
                                   const float sustainLevel)
{
    if (index < 0 || index >= kMaxVoices) {
        return;
    }

    voices_[index].attackMs.store(attackMs, std::memory_order_relaxed);
    voices_[index].decayMs.store(decayMs, std::memory_order_relaxed);
    voices_[index].durationMs.store(durationMs,
                                    std::memory_order_relaxed);
    voices_[index].releaseMs.store(releaseMs,
                                   std::memory_order_relaxed);
    voices_[index].sustainLevel.store(sustainLevel,
                                      std::memory_order_relaxed);
}

void AudioEngine::clearAllConnectionWaveformTaps()
{
    numConnectionTaps_.store(0, std::memory_order_relaxed);
    connectionKeyToTapIndex_.clear();

    for (int i = 0; i < kMaxConnectionTaps; ++i) {
        connectionTaps_[i].kind.store(0, std::memory_order_relaxed);
        connectionTaps_[i].voiceIndex.store(-1,
                                            std::memory_order_relaxed);
    }
}

void AudioEngine::configureConnectionWaveformTap(
    const std::string& connectionKey, const ConnectionTapSourceKind kind,
    const int voiceIndex)
{
    if (kind == ConnectionTapSourceKind::kNone) {
        return;
    }

    if ((kind == ConnectionTapSourceKind::kVoicePre ||
         kind == ConnectionTapSourceKind::kVoicePost) &&
        (voiceIndex < 0 || voiceIndex >= kMaxVoices)) {
        return;
    }

    int index = -1;
    const auto it = connectionKeyToTapIndex_.find(connectionKey);
    if (it != connectionKeyToTapIndex_.end()) {
        index = it->second;
    } else {
        const int current =
            numConnectionTaps_.load(std::memory_order_relaxed);
        if (current >= kMaxConnectionTaps) {
            return;
        }
        index = current;
        numConnectionTaps_.store(current + 1,
                                 std::memory_order_relaxed);
        connectionKeyToTapIndex_.emplace(connectionKey, index);
    }

    connectionTaps_[index].voiceIndex.store(voiceIndex,
                                            std::memory_order_relaxed);
    connectionTaps_[index].kind.store(
        static_cast<int>(kind), std::memory_order_relaxed);
}

void AudioEngine::getWaveformSnapshot(float* dst, const int numPoints,
                                      const double windowSeconds)
{
    if (dst == nullptr || numPoints <= 0 || sampleRate_ <= 0.0) {
        return;
    }

    const int historySize = kWaveformHistorySize;
    const int writeIndex =
        waveformWriteIndex_.load(std::memory_order_relaxed);
    const int availableSamples =
        std::min(historySize, std::max(writeIndex, 0));

    if (availableSamples <= 0) {
        std::fill(dst, dst + numPoints, 0.0F);
        return;
    }

    double window = windowSeconds;
    if (window <= 0.0) {
        window = 0.05;  // Default to ~50 ms.
    }

    int windowSamples = static_cast<int>(window * sampleRate_);
    windowSamples = std::max(1, std::min(windowSamples, availableSamples));

    const int startIndex =
        (writeIndex - windowSamples + historySize * 4) % historySize;

    // Downsample the requested window into `numPoints` evenly spaced
    // samples. The UI can then normalise and map these to screen-space.
    const int points = std::min(numPoints, windowSamples);
    const float denom = static_cast<float>(std::max(points - 1, 1));

    for (int i = 0; i < points; ++i) {
        const float t = static_cast<float>(i) / denom;
        const int offset = static_cast<int>(t * static_cast<float>(windowSamples - 1));
        const int bufIndex =
            (startIndex + offset + historySize) % historySize;
        dst[i] = waveformBuffer_[bufIndex];
    }

    // If numPoints > points (e.g. extremely small window), pad the
    // remainder with the last value to keep the curve continuous.
    for (int i = points; i < numPoints; ++i) {
        dst[i] = dst[points - 1];
    }
}

void AudioEngine::getConnectionWaveformSnapshot(
    const std::string& connectionKey, float* dst, const int numPoints,
    const double windowSeconds)
{
    if (dst == nullptr || numPoints <= 0 || sampleRate_ <= 0.0) {
        return;
    }

    const auto it = connectionKeyToTapIndex_.find(connectionKey);
    if (it == connectionKeyToTapIndex_.end()) {
        std::fill(dst, dst + numPoints, 0.0F);
        return;
    }

    const int tapIndex = it->second;
    if (tapIndex < 0 || tapIndex >= kMaxConnectionTaps) {
        std::fill(dst, dst + numPoints, 0.0F);
        return;
    }

    const int historySize = kWaveformHistorySize;
    const int writeIndex =
        waveformWriteIndex_.load(std::memory_order_relaxed);
    const int availableSamples =
        std::min(historySize, std::max(writeIndex, 0));

    if (availableSamples <= 0) {
        std::fill(dst, dst + numPoints, 0.0F);
        return;
    }

    double window = windowSeconds;
    if (window <= 0.0) {
        window = 0.05;  // Default to ~50 ms.
    }

    int windowSamples = static_cast<int>(window * sampleRate_);
    windowSamples =
        std::max(1, std::min(windowSamples, availableSamples));

    const int startIndex =
        (writeIndex - windowSamples + historySize * 4) % historySize;

    const int points = std::min(numPoints, windowSamples);
    const float denom = static_cast<float>(std::max(points - 1, 1));

    for (int i = 0; i < points; ++i) {
        const float t = static_cast<float>(i) / denom;
        const int offset = static_cast<int>(
            t * static_cast<float>(windowSamples - 1));
        const int bufIndex =
            (startIndex + offset + historySize) % historySize;
        dst[i] = connectionWaveformBuffers_[tapIndex][bufIndex];
    }

    for (int i = points; i < numPoints; ++i) {
        dst[i] = dst[points - 1];
    }
}

void AudioEngine::getLoopModuleWaveformSnapshot(
    const std::string& moduleId, float* dst, const int numPoints,
    const double windowSeconds)
{
    if (dst == nullptr || numPoints <= 0 || sampleRate_ <= 0.0) {
        return;
    }

    const auto it = loopModuleToWaveformIndex_.find(moduleId);
    if (it == loopModuleToWaveformIndex_.end()) {
        std::fill(dst, dst + numPoints, 0.0F);
        return;
    }

    const int wfIndex = it->second;
    if (wfIndex < 0 || wfIndex >= kMaxLoopWaveforms) {
        std::fill(dst, dst + numPoints, 0.0F);
        return;
    }

    const int historySize = kWaveformHistorySize;
    const int writeIndex =
        waveformWriteIndex_.load(std::memory_order_relaxed);
    const int availableSamples =
        std::min(historySize, std::max(writeIndex, 0));

    if (availableSamples <= 0) {
        std::fill(dst, dst + numPoints, 0.0F);
        return;
    }

    double window = windowSeconds;
    if (window <= 0.0) {
        window = 0.05;  // Default to ~50 ms.
    }

    int windowSamples = static_cast<int>(window * sampleRate_);
    windowSamples =
        std::max(1, std::min(windowSamples, availableSamples));

    const int startIndex =
        (writeIndex - windowSamples + historySize * 4) % historySize;

    const int points = std::min(numPoints, windowSamples);
    const float denom = static_cast<float>(std::max(points - 1, 1));

    for (int i = 0; i < points; ++i) {
        const float t = static_cast<float>(i) / denom;
        const int offset = static_cast<int>(
            t * static_cast<float>(windowSamples - 1));
        const int bufIndex =
            (startIndex + offset + historySize) % historySize;
        dst[i] = loopWaveformBuffers_[wfIndex][bufIndex];
    }

    for (int i = points; i < numPoints; ++i) {
        dst[i] = dst[points - 1];
    }
}

void AudioEngine::rebuildAudioGraphFromScene(const rectai::Scene& scene)
{
    if (audioGraph_ != nullptr) {
        audioGraph_->RebuildFromScene(scene);
    }
}

void AudioEngine::setVoiceFilter(const int index, const int mode,
                                 const double cutoffHz, const float q)
{
    if (index < 0 || index >= kMaxVoices) {
        return;
    }

    const int clampedMode = juce::jlimit(0, 3, mode);

    // Resonance must be strictly > 0 for the JUCE filter; clamp
    // to a musically useful range to prevent extreme peaks and
    // audio amplification when switching filter modes.
    const float qClamped = juce::jlimit(0.1F, 15.0F, q);

    const double sr = sampleRate_ > 0.0 ? sampleRate_ : 44100.0;
    const double nyquist = 0.5 * sr;
    double fc = cutoffHz;
    if (fc <= 0.0) {
        fc = 0.0;
    }
    if (fc >= nyquist) {
        // JUCE requires cutoff < Nyquist, not <=.
        fc = nyquist * 0.99;
    }

    voices_[index].filterMode.store(clampedMode, std::memory_order_relaxed);
    voices_[index].filterCutoffHz.store(fc, std::memory_order_relaxed);
    voices_[index].filterQ.store(qClamped, std::memory_order_relaxed);

    // If the filter is disabled or has an invalid cutoff, leave it
    // effectively bypassed. We avoid resetting here to preserve
    // continuity when parameters are toggled.
    if (clampedMode == 0 || fc <= 0.0) {
        return;
    }

    switch (clampedMode) {
        case 1: // Low-pass
            filters_[index].setType(
                juce::dsp::StateVariableTPTFilterType::lowpass);
            break;
        case 2: // Band-pass
            filters_[index].setType(
                juce::dsp::StateVariableTPTFilterType::bandpass);
            break;
        case 3: // High-pass
            filters_[index].setType(
                juce::dsp::StateVariableTPTFilterType::highpass);
            break;
        default:
            return;
    }

    filters_[index].setCutoffFrequency(static_cast<float>(fc));
    filters_[index].setResonance(qClamped);
}

void AudioEngine::getVoiceWaveformSnapshot(const int voiceIndex,
                                           float* dst,
                                           const int numPoints,
                                           const double windowSeconds)
{
    if (dst == nullptr || numPoints <= 0 || sampleRate_ <= 0.0 ||
        voiceIndex < 0 || voiceIndex >= kMaxVoices) {
        return;
    }

    const int historySize = kWaveformHistorySize;
    const int writeIndex =
        waveformWriteIndex_.load(std::memory_order_relaxed);
    const int availableSamples =
        std::min(historySize, std::max(writeIndex, 0));

    if (availableSamples <= 0) {
        std::fill(dst, dst + numPoints, 0.0F);
        return;
    }

    double window = windowSeconds;
    if (window <= 0.0) {
        window = 0.05;  // Default to ~50 ms.
    }

    int windowSamples = static_cast<int>(window * sampleRate_);
    windowSamples = std::max(1, std::min(windowSamples, availableSamples));

    const int startIndex =
        (writeIndex - windowSamples + historySize * 4) % historySize;

    const int points = std::min(numPoints, windowSamples);
    const float denom = static_cast<float>(std::max(points - 1, 1));

    for (int i = 0; i < points; ++i) {
        const float t = static_cast<float>(i) / denom;
        const int offset = static_cast<int>(
            t * static_cast<float>(windowSamples - 1));
        const int bufIndex =
            (startIndex + offset + historySize) % historySize;
        dst[i] =
            voicePreFilterWaveformBuffer_[voiceIndex][bufIndex];
    }

    for (int i = points; i < numPoints; ++i) {
        dst[i] = dst[points - 1];
    }
}

void AudioEngine::getVoiceFilteredWaveformSnapshot(const int voiceIndex,
                                                   float* dst,
                                                   const int numPoints,
                                                   const double windowSeconds)
{
    if (dst == nullptr || numPoints <= 0 || sampleRate_ <= 0.0 ||
        voiceIndex < 0 || voiceIndex >= kMaxVoices) {
        return;
    }

    const int historySize = kWaveformHistorySize;
    const int writeIndex =
        waveformWriteIndex_.load(std::memory_order_relaxed);
    const int availableSamples =
        std::min(historySize, std::max(writeIndex, 0));

    if (availableSamples <= 0) {
        std::fill(dst, dst + numPoints, 0.0F);
        return;
    }

    double window = windowSeconds;
    if (window <= 0.0) {
        window = 0.05;  // Default to ~50 ms.
    }

    int windowSamples = static_cast<int>(window * sampleRate_);
    windowSamples = std::max(1, std::min(windowSamples, availableSamples));

    const int startIndex =
        (writeIndex - windowSamples + historySize * 4) % historySize;

    const int points = std::min(numPoints, windowSamples);
    const float denom = static_cast<float>(std::max(points - 1, 1));

    for (int i = 0; i < points; ++i) {
        const float t = static_cast<float>(i) / denom;
        const int offset = static_cast<int>(
            t * static_cast<float>(windowSamples - 1));
        const int bufIndex =
            (startIndex + offset + historySize) % historySize;
        dst[i] =
            voicePostFilterWaveformBuffer_[voiceIndex][bufIndex];
    }

    for (int i = points; i < numPoints; ++i) {
        dst[i] = dst[points - 1];
    }
}
