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
    // preset will be configured later from the UI thread.
    sampleplaySynth_ = std::make_unique<rectai::SampleplaySynth>();

    // Initialise the logical audio graph so that callers can safely
    // query it even before the Scene has been rebuilt for the first
    // time. The initial graph is simply empty.
    audioGraph_ = std::make_unique<rectai::AudioGraph>();

    // Initialise per-voice RNG state for noise waveforms. The
    // xorshift32 algorithm used in the audio callback treats an
    // all-zero state as a fixed point that would generate silence,
    // so ensure that each voice starts from a distinct non-zero
    // seed.
    for (int v = 0; v < kMaxVoices; ++v) {
        std::uint32_t seed = static_cast<std::uint32_t>(0x12345678u) +
                             static_cast<std::uint32_t>(v) *
                                 static_cast<std::uint32_t>(0x9E3779B9u);
        if (seed == 0u) {
            seed = 1u;
        }
        noiseState_[v] = seed;
    }

    // Initialise delay line buffers for the global Delay FX bus.
    delayBufferL_.assign(static_cast<std::size_t>(kMaxDelaySamples), 0.0F);
    delayBufferR_.assign(static_cast<std::size_t>(kMaxDelaySamples), 0.0F);
    delayWriteIndex_ = 0;

    // Register basic audio formats once; used for Loop module
    // sample decoding (WAV/FLAC/Ogg/Opus, depending on JUCE
    // configuration).
    loopFormatManager_.registerBasicFormats();
}

AudioEngine::~AudioEngine()
{
    shutdown();
}

void AudioEngine::shutdown()
{
    if (isShutdown_) {
        return;
    }

    isShutdown_ = true;

    // Detach this callback from the AudioDeviceManager so that no
    // further audio callbacks are issued while the engine is being
    // torn down. The AudioDeviceManager (owned by AudioEngine) will
    // close the underlying device in its own destructor.
    deviceManager_.removeAudioCallback(this);
}

void AudioEngine::audioDeviceAboutToStart(juce::AudioIODevice* device)
{
    const double sr =
        (device != nullptr) ? device->getCurrentSampleRate() : 44100.0;
    sampleRate_ = sr > 0.0 ? sr : 44100.0;

    visualVoices_.setSampleRate(sampleRate_);
    visualVoices_.reset();

    // Keep the pre-filter oscillator voices in sync with the same
    // sample rate and reset their history so that pre/post snapshots
    // start from a consistent state on device (re)start.
    oscPreVoices_.setSampleRate(sampleRate_);
    oscPreVoices_.reset();

    // Reset the global Delay / Reverb FX state so that delay lines
    // and reverb tails start from silence on device (re)start.
    std::fill(delayBufferL_.begin(), delayBufferL_.end(), 0.0F);
    std::fill(delayBufferR_.begin(), delayBufferR_.end(), 0.0F);
    delayWriteIndex_ = 0;
    reverb_.reset();
    delayTailSamplesRemaining_.store(0, std::memory_order_relaxed);
    reverbTailSamplesRemaining_.store(0, std::memory_order_relaxed);

    // Reset transport state so that loop alignment and beat-synced
    // visuals start from a known position when the device starts.
    transportBeatsInternal_ = 0.0;
    transportBeatsAudio_.store(0.0, std::memory_order_relaxed);
    hasPendingOutput_.store(false, std::memory_order_relaxed);
}

void AudioEngine::audioDeviceStopped()
{
    hasPendingOutput_.store(false, std::memory_order_relaxed);
}

void AudioEngine::audioDeviceIOCallbackWithContext(
    const float* const* inputChannelData,
    const int numInputChannels,
    float* const* outputChannelData,
    const int numOutputChannels,
    const int numSamples,
    const juce::AudioIODeviceCallbackContext& context)
{
    juce::ignoreUnused(inputChannelData, numInputChannels, context);

    // If the device has not been started properly yet, clear the
    // outputs and bail out early. This protects against divide-by-
    // zero when computing time deltas and keeps the engine quiet
    // while the device is being reconfigured.
    if (sampleRate_ <= 0.0) {
        for (int channel = 0; channel < numOutputChannels; ++channel) {
            if (auto* buffer = outputChannelData[channel]) {
                std::fill(buffer, buffer + numSamples, 0.0F);
            }
        }
        return;
    }

    // Advance the global transport position in beats using the
    // current BPM and sample rate. This must happen before the
    // early-return path below so that tempo-synchronised visuals
    // and loop alignment keep progressing even when the scene is
    // effectively silent (for example, when all modules are docked
    // or muted but the device is still running).
    const double bpmForTransport = static_cast<double>(
        loopGlobalBpm_.load(std::memory_order_relaxed));
    if (bpmForTransport > 0.0) {
        const double secondsPerBeat = 60.0 / bpmForTransport;
        if (secondsPerBeat > 0.0) {
            const double blockSeconds = static_cast<double>(numSamples) /
                                        sampleRate_;
            const double beatDelta = blockSeconds / secondsPerBeat;
            transportBeatsInternal_ += beatDelta;
        }
    }

    transportBeatsAudio_.store(transportBeatsInternal_,
                               std::memory_order_relaxed);

    const bool hadPendingOutput =
        hasPendingOutput_.load(std::memory_order_relaxed);
    const bool shouldProcess =
        processingRequested_.load(std::memory_order_relaxed) ||
        hadPendingOutput;

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

    const int voiceCount =
        std::min(numVoices_.load(std::memory_order_relaxed), kMaxVoices);

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

    static bool loggedLoopSnapshotOnce = false;
    if (!loggedLoopSnapshotOnce && loopSnapshot && !loopSnapshot->empty()) {
        loggedLoopSnapshotOnce = true;
        juce::String msg("[rectai-core] Loop: snapshot contains ");
        msg += juce::String(static_cast<int>(loopSnapshot->size()));
        msg += " instances:";
        for (const auto& pair : *loopSnapshot) {
            msg += " ";
            msg += juce::String(pair.first);
        }
        juce::Logger::writeToLog(msg);
    }

    // Use the audio-driven transport position (already updated at the
    // start of this callback) for Loop alignment so that Loop
    // playback stays in phase with the same master clock used by
    // Sequencer timing and beat pulses.
    const double loopBeats = transportBeatsInternal_;

    // Current global BPM used for tempo-synchronised Loop playback.
    const double bpm = static_cast<double>(
        loopGlobalBpm_.load(std::memory_order_relaxed));

    bool blockHasNonZeroOutput = false;

    // Precompute Delay / Reverb FX buses for this block.
    const bool delayBusActive =
        delayBus_.active.load(std::memory_order_relaxed) != 0;
    const bool reverbBusActive =
        reverbBus_.active.load(std::memory_order_relaxed) != 0;

    const float delayNormRaw =
        delayBus_.delayNormalised.load(std::memory_order_relaxed);
    const float delayFeedbackRaw =
        delayBus_.feedback.load(std::memory_order_relaxed);
    const float delayWetRaw =
        delayBus_.wetGain.load(std::memory_order_relaxed);

    const float reverbAmountRaw =
        reverbBus_.reverbAmount.load(std::memory_order_relaxed);
    const float reverbWetRaw =
        reverbBus_.wetGain.load(std::memory_order_relaxed);

    int delaySamplesForBlock = 0;
    float delayFeedbackForBlock = 0.0F;
    float delayWetGainForBlock = 0.0F;

    float reverbAmountForBlock = 0.0F;
    float reverbWetGainForBlock = 0.0F;

    const bool delayApplyGlobal =
        delayBusActive &&
        delayBus_.applyGlobal.load(std::memory_order_relaxed) != 0;
    const bool reverbApplyGlobal =
        reverbBusActive &&
        reverbBus_.applyGlobal.load(std::memory_order_relaxed) != 0;

    const bool delayTargetsSampleplay =
        delayBusActive &&
        delayBus_.includeSampleplay.load(std::memory_order_relaxed) != 0;
    const bool reverbTargetsSampleplay =
        reverbBusActive &&
        reverbBus_.includeSampleplay.load(std::memory_order_relaxed) != 0;

    const std::uint32_t delayVoiceMask =
        delayBus_.voiceMask.load(std::memory_order_relaxed);
    const std::uint32_t reverbVoiceMask =
        reverbBus_.voiceMask.load(std::memory_order_relaxed);

    const bool delayTargetsVoice =
        delayBusActive && delayVoiceMask != 0u;
    const bool reverbTargetsVoice =
        reverbBusActive && reverbVoiceMask != 0u;

    const int configuredDelayLoopTargets =
        delayBus_.loopTargetCount.load(std::memory_order_relaxed);
    const int delayLoopTargetCount = juce::jlimit(
        0, DelayTargetConfig::kMaxLoopTargets, configuredDelayLoopTargets);
    std::array<std::uint64_t, DelayTargetConfig::kMaxLoopTargets>
        delayLoopHashes{};
    for (int i = 0; i < delayLoopTargetCount; ++i) {
        delayLoopHashes[static_cast<std::size_t>(i)] =
            delayBus_.loopTargetHashes[static_cast<std::size_t>(i)].load(
                std::memory_order_relaxed);
    }
    const bool delayTargetsLoop =
        delayBusActive && delayLoopTargetCount > 0;

    const int configuredReverbLoopTargets =
        reverbBus_.loopTargetCount.load(std::memory_order_relaxed);
    const int reverbLoopTargetCount = juce::jlimit(
        0, DelayTargetConfig::kMaxLoopTargets, configuredReverbLoopTargets);
    std::array<std::uint64_t, DelayTargetConfig::kMaxLoopTargets>
        reverbLoopHashes{};
    for (int i = 0; i < reverbLoopTargetCount; ++i) {
        reverbLoopHashes[static_cast<std::size_t>(i)] =
            reverbBus_.loopTargetHashes[static_cast<std::size_t>(i)].load(
                std::memory_order_relaxed);
    }
    const bool reverbTargetsLoop =
        reverbBusActive && reverbLoopTargetCount > 0;

    auto loopMatchesDelay = [&](const std::string& moduleId) -> bool {
        if (!delayTargetsLoop) {
            return false;
        }
        const std::uint64_t hash = rectai::HashModuleId(moduleId);
        for (int i = 0; i < delayLoopTargetCount; ++i) {
            if (delayLoopHashes[static_cast<std::size_t>(i)] == hash) {
                return true;
            }
        }
        return false;
    };

    auto loopMatchesReverb = [&](const std::string& moduleId) -> bool {
        if (!reverbTargetsLoop) {
            return false;
        }
        const std::uint64_t hash = rectai::HashModuleId(moduleId);
        for (int i = 0; i < reverbLoopTargetCount; ++i) {
            if (reverbLoopHashes[static_cast<std::size_t>(i)] == hash) {
                return true;
            }
        }
        return false;
    };

    int delayTailSamples =
        delayTailSamplesRemaining_.load(std::memory_order_relaxed);
    if (!delayBusActive) {
        delayTailSamples = 0;
    }

    int reverbTailSamples =
        reverbTailSamplesRemaining_.load(std::memory_order_relaxed);
    if (!reverbBusActive) {
        reverbTailSamples = 0;
    }

    if (delayBusActive) {
        const float clampedNorm =
            juce::jlimit(0.0F, 1.0F, delayNormRaw);
        int segment = static_cast<int>(clampedNorm * 8.0F);
        if (segment < 0) {
            segment = 0;
        } else if (segment > 7) {
            segment = 7;
        }

        static constexpr float kBeatMultipliers[8] = {
            1.0F / 32.0F,
            1.0F / 16.0F,
            1.0F / 8.0F,
            1.0F / 4.0F,
            3.0F / 8.0F,
            2.0F / 4.0F,
            4.0F / 4.0F,
            8.0F / 4.0F};

        const float bpmDelay =
            loopGlobalBpm_.load(std::memory_order_relaxed);
        double secondsPerBeat = 0.0;
        if (bpmDelay > 0.0F) {
            secondsPerBeat =
                60.0 / static_cast<double>(bpmDelay);
        }
        if (secondsPerBeat <= 0.0) {
            secondsPerBeat = 0.5;
        }

        const double delaySeconds = std::max(
            0.01,
            static_cast<double>(kBeatMultipliers[segment]) *
                secondsPerBeat);

        const double maxSamples =
            static_cast<double>(kMaxDelaySamples - 1);
        const int computedSamples = static_cast<int>(
            std::round(delaySeconds * sampleRate_));
        delaySamplesForBlock =
            juce::jlimit(1, static_cast<int>(maxSamples),
                         computedSamples);

        delayFeedbackForBlock =
            juce::jlimit(0.0F, 0.95F, delayFeedbackRaw);
        delayWetGainForBlock =
            juce::jlimit(0.0F, 1.0F, delayWetRaw);
    }

    if (reverbBusActive) {
        reverbAmountForBlock =
            juce::jlimit(0.0F, 1.0F, reverbAmountRaw);
        reverbWetGainForBlock =
            juce::jlimit(0.0F, 1.0F, reverbWetRaw);

        juce::Reverb::Parameters params;
        params.roomSize = 0.5F + 0.5F * reverbAmountForBlock;
        params.damping = 0.2F + 0.6F * reverbAmountForBlock;
        params.wetLevel = reverbAmountForBlock;
        params.dryLevel = 0.0F;
        params.width = 1.0F;
        params.freezeMode = 0.0F;
        reverb_.setParameters(params);
    }

    auto processDelayBusSample =
        [this, delayBusActive, delaySamplesForBlock,
         delayFeedbackForBlock, delayWetGainForBlock](float& left,
                                                      float& right) {
            if (!delayBusActive) {
                left = 0.0F;
                right = 0.0F;
                return;
            }

            if (delaySamplesForBlock <= 0 || delayBufferL_.empty() ||
                delayBufferR_.empty()) {
                left = 0.0F;
                right = 0.0F;
                return;
            }

            const float inputL = left;
            const float inputR = right;

            const int bufferSize =
                static_cast<int>(delayBufferL_.size());
            const int writeIndex = delayWriteIndex_;
            int readIndex = writeIndex - delaySamplesForBlock;
            if (readIndex < 0) {
                readIndex += bufferSize;
            }

            const float delayedL =
                delayBufferL_[static_cast<std::size_t>(readIndex)];
            const float delayedR =
                delayBufferR_[static_cast<std::size_t>(readIndex)];

            const float fb = delayFeedbackForBlock;

            const float wetL = delayedL * delayWetGainForBlock;
            const float wetR = delayedR * delayWetGainForBlock;

            const float writeL = inputL + delayedL * fb;
            const float writeR = inputR + delayedR * fb;

            delayBufferL_[static_cast<std::size_t>(writeIndex)] =
                writeL;
            delayBufferR_[static_cast<std::size_t>(writeIndex)] =
                writeR;

            int nextWrite = writeIndex + 1;
            if (nextWrite >= bufferSize) {
                nextWrite = 0;
            }
            delayWriteIndex_ = nextWrite;

            left = wetL;
            right = wetR;
        };

    auto processReverbBusSample =
        [this, reverbBusActive, reverbAmountForBlock,
         reverbWetGainForBlock](float& left, float& right) {
            if (!reverbBusActive) {
                left = 0.0F;
                right = 0.0F;
                return;
            }

            float wetL = left;
            float wetR = right;
            reverb_.processStereo(&wetL, &wetR, 1);
            wetL *= reverbWetGainForBlock;
            wetR *= reverbWetGainForBlock;
            left = wetL;
            right = wetR;
        };

    for (int sample = 0; sample < numSamples; ++sample) {
        float oscMonoDry = 0.0F;
        float oscDelayL = 0.0F;
        float oscDelayR = 0.0F;
        float oscReverbL = 0.0F;
        float oscReverbR = 0.0F;
        bool delayHadSourceInputThisSample = false;
        bool reverbHadSourceInputThisSample = false;

        // Optionally run the Sampleplay stereo path through a
        // StateVariableTPTFilter so that downstream Filter modules
        // can affect SoundFont audio when connected. We keep the
        // pre-filter mono sample (raw) and the post-filter mono
        // sample in separate history buffers so that connections
        // Sampleplay → X can display the original SoundFont
        // waveform while radials aguas abajo reflejan la señal ya
        // procesada.
        auto& spBus =
            busFilters_[kBusFilterSampleplay];
        const double spCutoff =
            spBus.cutoffHz.load(std::memory_order_relaxed);
        const std::size_t spIdx =
            static_cast<std::size_t>(sample);
        const float rawSampleplayL =
            sampleplayLeft_[spIdx];  // after global gain
        const float rawSampleplayR =
            sampleplayRight_[spIdx];

        float filteredSampleplayL = rawSampleplayL;
        float filteredSampleplayR = rawSampleplayR;
        if (spCutoff > 0.0) {
            filteredSampleplayL =
                spBus.filterL.processSample(0, rawSampleplayL);
            filteredSampleplayR =
                spBus.filterR.processSample(0, rawSampleplayR);
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

                // Compute per-voice samples before and after the
                // per-voice filter, including both envelope and
                // level. The pre-filter value feeds the dedicated
                // oscillator pre-history used for Osc → Filter
                // visuals, while the post-filter value feeds the
                // unified visual Voices container and the actual
                // mono mix.
                const float voiceSamplePre =
                    static_cast<float>(levelForMix * envAmp) * raw;
                const float voiceSamplePost =
                    static_cast<float>(levelForMix * envAmp) * s;

                // Pre-filter visual history for Oscillator voices.
                oscPreVoices_.writeSamples(v, &voiceSamplePre, 1);

                // Post-filter visual history for Oscillator voices
                // in the unified container. Voice ids
                // [0, kMaxVoices) are reserved for oscillator
                // voices; Loop modules and other producers are
                // placed after that range.
                visualVoices_.writeSamples(v, &voiceSamplePost, 1);

                std::uint32_t voiceBit = 0u;
                if (v >= 0 && v < 32) {
                    voiceBit = 1u
                               << static_cast<std::uint32_t>(v);
                }
                const bool voiceMatchesDelay =
                    delayTargetsVoice && voiceBit != 0u &&
                    ((delayVoiceMask & voiceBit) != 0u);
                const bool voiceMatchesReverb =
                    reverbTargetsVoice && voiceBit != 0u &&
                    ((reverbVoiceMask & voiceBit) != 0u);

                if (voiceMatchesDelay) {
                    float delayL = voiceSamplePost;
                    float delayR = voiceSamplePost;
                    processDelayBusSample(delayL, delayR);
                    delayHadSourceInputThisSample = true;
                    oscDelayL += delayL;
                    oscDelayR += delayR;
                }

                if (voiceMatchesReverb) {
                    float reverbL = voiceSamplePost;
                    float reverbR = voiceSamplePost;
                    processReverbBusSample(reverbL, reverbR);
                    reverbHadSourceInputThisSample = true;
                    oscReverbL += reverbL;
                    oscReverbR += reverbR;
                }

                // Always mix the post-filter signal into the dry bus so
                // that delay/reverb operate as pure send effects.
                oscMonoDry += voiceSamplePost;
            }

            float oscMixedLeft = oscMonoDry;
            float oscMixedRight = oscMonoDry;
            if (delayTargetsVoice) {
                oscMixedLeft += oscDelayL;
                oscMixedRight += oscDelayR;
            }
            if (reverbTargetsVoice) {
                oscMixedLeft += oscReverbL;
                oscMixedRight += oscReverbR;
            }

            oscMixedLeft = juce::jlimit(-0.9F, 0.9F, oscMixedLeft);
            oscMixedRight = juce::jlimit(-0.9F, 0.9F, oscMixedRight);

            const float sampleplayMonoRaw = rawSampleplayL;

            // Mirror the global Sampleplay mono path into a
            // dedicated visual voice so that Sampleplay-based
            // modules have a stable waveform source in the unified
            // Voices container. Per-module Sampleplay visuals will
            // map onto this shared voice in later phases.
            visualVoices_.writeSamples(kVisualVoiceIdSampleplayRaw,
                                       &sampleplayMonoRaw, 1);

            float sampleplayDelayWetL = 0.0F;
            float sampleplayDelayWetR = 0.0F;
            if (delayTargetsSampleplay) {
                sampleplayDelayWetL = filteredSampleplayL;
                sampleplayDelayWetR = filteredSampleplayR;
                processDelayBusSample(sampleplayDelayWetL,
                                      sampleplayDelayWetR);
                if (delayBusActive) {
                    delayHadSourceInputThisSample = true;
                }
            }

            float sampleplayReverbWetL = 0.0F;
            float sampleplayReverbWetR = 0.0F;
            if (reverbTargetsSampleplay) {
                sampleplayReverbWetL = filteredSampleplayL;
                sampleplayReverbWetR = filteredSampleplayR;
                processReverbBusSample(sampleplayReverbWetL,
                                       sampleplayReverbWetR);
                if (reverbBusActive) {
                    reverbHadSourceInputThisSample = true;
                }
            }

            float leftOut = filteredSampleplayL + oscMixedLeft +
                            sampleplayDelayWetL +
                            sampleplayReverbWetL;
            float rightOut = filteredSampleplayR + oscMixedRight +
                             sampleplayDelayWetR +
                             sampleplayReverbWetR;
            float loopMixedL = 0.0F;
            float loopMixedR = 0.0F;
            float loopDelaySumL = 0.0F;
            float loopDelaySumR = 0.0F;
            float loopReverbSumL = 0.0F;
            float loopReverbSumR = 0.0F;

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
                    const std::string& loopModuleId = pair.first;
                    auto& instance = pair.second;
                    const int slotIndex = instance.selectedIndex.load(
                        std::memory_order_relaxed);
                    const float loopGainTarget = instance.gain.load(
                        std::memory_order_relaxed);
                    const bool matchesDelayLoop =
                        loopMatchesDelay(loopModuleId);
                    const bool matchesReverbLoop =
                        loopMatchesReverb(loopModuleId);

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

                            const double srLocal =
                                sampleRate_ > 0.0 ? sampleRate_
                                                  : 44100.0;
                            const double dt =
                                (srLocal > 0.0) ? (1.0 / srLocal)
                                                : 0.0;

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
                        const int wfIndex = instance.visualWaveformIndex;
                        if (wfIndex >= 0 &&
                            wfIndex < kMaxLoopWaveforms) {
                            const int loopVoiceId =
                                kMaxVoices + wfIndex;
                            const float loopVisualSample = monoWithGain;
                            visualVoices_.writeSamples(loopVoiceId,
                                                        &loopVisualSample,
                                                        1);
                        }

                        if (finalGain > 0.0F) {
                            const bool routeThroughFilter =
                                instance.routeThroughFilter.load(
                                    std::memory_order_relaxed);
                            if (routeThroughFilter) {
                                loopMixedL += monoWithGain;
                                loopMixedR += r * finalGain;
                            } else {
                                leftOut += monoWithGain;
                                rightOut += r * finalGain;
                            }

                            if (matchesDelayLoop && delayBusActive) {
                                loopDelaySumL += monoWithGain;
                                loopDelaySumR += r * finalGain;
                            }

                            if (matchesReverbLoop && reverbBusActive) {
                                loopReverbSumL += monoWithGain;
                                loopReverbSumR += r * finalGain;
                            }
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

            // Optionally run the summed Loop output through the
            // Loop bus filter so that Loop → Filter connections in
            // the Scene can shape loop audio without affecting other
            // paths.
            auto& loopBus = busFilters_[kBusFilterLoop];
            const double loopCutoff =
                loopBus.cutoffHz.load(std::memory_order_relaxed);
            float loopFilteredL = loopMixedL;
            float loopFilteredR = loopMixedR;
            if (loopCutoff > 0.0) {
                loopFilteredL =
                    loopBus.filterL.processSample(0, loopFilteredL);
                loopFilteredR =
                    loopBus.filterR.processSample(0, loopFilteredR);
            }

            // Expose the post-filter Loop bus output as a dedicated
            // visual voice. This represents the signal leaving the
            // Loop → Filter chain and will be used by FX/module
            // visuals where Loop audio is upstream.
            visualVoices_.writeSamples(kVisualVoiceIdLoopBus,
                                       &loopFilteredL, 1);

            leftOut += loopFilteredL;
            rightOut += loopFilteredR;

            if (delayTargetsLoop &&
                (loopDelaySumL != 0.0F || loopDelaySumR != 0.0F)) {
                float delayL = loopDelaySumL;
                float delayR = loopDelaySumR;
                processDelayBusSample(delayL, delayR);
                if (delayBusActive) {
                    delayHadSourceInputThisSample = true;
                }
                leftOut += delayL;
                rightOut += delayR;
            }

            if (reverbTargetsLoop &&
                (loopReverbSumL != 0.0F || loopReverbSumR != 0.0F)) {
                float reverbL = loopReverbSumL;
                float reverbR = loopReverbSumR;
                processReverbBusSample(reverbL, reverbR);
                if (reverbBusActive) {
                    reverbHadSourceInputThisSample = true;
                }
                leftOut += reverbL;
                rightOut += reverbR;
            }

            const float globalFxInputL = leftOut;
            const float globalFxInputR = rightOut;

            if (delayApplyGlobal) {
                float delayL = globalFxInputL;
                float delayR = globalFxInputR;
                processDelayBusSample(delayL, delayR);
                if (delayBusActive) {
                    delayHadSourceInputThisSample = true;
                }
                leftOut += delayL;
                rightOut += delayR;
            }

            if (reverbApplyGlobal) {
                float reverbL = globalFxInputL;
                float reverbR = globalFxInputR;
                processReverbBusSample(reverbL, reverbR);
                if (reverbBusActive) {
                    reverbHadSourceInputThisSample = true;
                }
                leftOut += reverbL;
                rightOut += reverbR;
            }

            if (!delayHadSourceInputThisSample && delayBusActive &&
                delayTailSamples > 0) {
                float tailL = 0.0F;
                float tailR = 0.0F;
                processDelayBusSample(tailL, tailR);
                leftOut += tailL;
                rightOut += tailR;
            }

            if (!reverbHadSourceInputThisSample && reverbBusActive &&
                reverbTailSamples > 0) {
                float tailL = 0.0F;
                float tailR = 0.0F;
                processReverbBusSample(tailL, tailR);
                leftOut += tailL;
                rightOut += tailR;
            }

            if (delayHadSourceInputThisSample) {
                delayTailSamples = kDelayTailHoldSamples;
            } else if (delayTailSamples > 0) {
                --delayTailSamples;
            }

            if (reverbHadSourceInputThisSample) {
                reverbTailSamples = kDelayTailHoldSamples;
            } else if (reverbTailSamples > 0) {
                --reverbTailSamples;
            }

            leftOut = juce::jlimit(-0.9F, 0.9F, leftOut);
            rightOut = juce::jlimit(-0.9F, 0.9F, rightOut);

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
            const float sampleplayDryL =
                sampleplayLeft_[static_cast<std::size_t>(sample)];
            const float sampleplayDryR =
                sampleplayRight_[static_cast<std::size_t>(sample)];
            float sampleplayDelayWetL = 0.0F;
            float sampleplayDelayWetR = 0.0F;
            if (delayTargetsSampleplay) {
                sampleplayDelayWetL = sampleplayDryL;
                sampleplayDelayWetR = sampleplayDryR;
                processDelayBusSample(sampleplayDelayWetL,
                                      sampleplayDelayWetR);
                if (delayBusActive) {
                    delayHadSourceInputThisSample = true;
                }
            }

            float sampleplayReverbWetL = 0.0F;
            float sampleplayReverbWetR = 0.0F;
            if (reverbTargetsSampleplay) {
                sampleplayReverbWetL = sampleplayDryL;
                sampleplayReverbWetR = sampleplayDryR;
                processReverbBusSample(sampleplayReverbWetL,
                                       sampleplayReverbWetR);
                if (reverbBusActive) {
                    reverbHadSourceInputThisSample = true;
                }
            }

            float leftOut = sampleplayDryL + sampleplayDelayWetL +
                            sampleplayReverbWetL;
            float rightOut = sampleplayDryR + sampleplayDelayWetR +
                             sampleplayReverbWetR;
            float loopMixedL = 0.0F;
            float loopMixedR = 0.0F;
            float loopDelaySumL = 0.0F;
            float loopDelaySumR = 0.0F;
            float loopReverbSumL = 0.0F;
            float loopReverbSumR = 0.0F;
            const float sampleplayMonoRaw = rawSampleplayL;

            // Mirror the global Sampleplay mono path into the
            // dedicated visual voice even in scenes without active
            // oscillator voices so that Sampleplay-only patches have
            // a consistent waveform history in the Voices container.
            visualVoices_.writeSamples(kVisualVoiceIdSampleplayRaw,
                                       &sampleplayMonoRaw, 1);

            // Mix Loop modules even when there are no oscillator
            // voices so that scenes consisting only of LoopModule
            // still render correctly. As in the oscillator branch,
            // align the selected slot's playback position from the
            // global loop beat counter when switching samples so that
            // different loops remain phase-locked to the transport.
            if (loopSnapshot && !loopSnapshot->empty()) {
                for (auto& pair : *loopSnapshot) {
                    const std::string& loopModuleId = pair.first;
                    auto& instance = pair.second;
                    const int slotIndex = instance.selectedIndex.load(
                        std::memory_order_relaxed);
                    const float loopGainTarget = instance.gain.load(
                        std::memory_order_relaxed);
                    const bool matchesDelayLoop =
                        loopMatchesDelay(loopModuleId);
                    const bool matchesReverbLoop =
                        loopMatchesReverb(loopModuleId);

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

                            const double srLocal =
                                sampleRate_ > 0.0 ? sampleRate_
                                                  : 44100.0;
                            const double dt =
                                (srLocal > 0.0) ? (1.0 / srLocal)
                                                : 0.0;

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
                        const int wfIndex = instance.visualWaveformIndex;
                        if (wfIndex >= 0 &&
                            wfIndex < kMaxLoopWaveforms) {
                            const int loopVoiceId =
                                kMaxVoices + wfIndex;
                            const float loopVisualSample = monoWithGain;
                            visualVoices_.writeSamples(loopVoiceId,
                                                        &loopVisualSample,
                                                        1);
                        }

                        if (finalGain > 0.0F) {
                            const bool routeThroughFilter =
                                instance.routeThroughFilter.load(
                                    std::memory_order_relaxed);
                            if (routeThroughFilter) {
                                loopMixedL += monoWithGain;
                                loopMixedR += r * finalGain;
                            } else {
                                leftOut += monoWithGain;
                                rightOut += r * finalGain;
                            }

                            if (matchesDelayLoop && delayBusActive) {
                                loopDelaySumL += monoWithGain;
                                loopDelaySumR += r * finalGain;
                            }

                            if (matchesReverbLoop && reverbBusActive) {
                                loopReverbSumL += monoWithGain;
                                loopReverbSumR += r * finalGain;
                            }
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

            // Optionally process the summed Loop output through the
            // Loop bus filter so that Loop → Filter connections
            // affect loop audio even in scenes without active
            // generators.
            auto& loopBus = busFilters_[kBusFilterLoop];
            const double loopCutoff =
                loopBus.cutoffHz.load(std::memory_order_relaxed);
            float loopFilteredL = loopMixedL;
            float loopFilteredR = loopMixedR;
            if (loopCutoff > 0.0) {
                loopFilteredL =
                    loopBus.filterL.processSample(0, loopFilteredL);
                loopFilteredR =
                    loopBus.filterR.processSample(0, loopFilteredR);
            }

            // Keep a dedicated visual voice for the Loop bus output
            // so that Loop-only scenes and Loop→Filter chains have a
            // stable post-filter waveform source.
            visualVoices_.writeSamples(kVisualVoiceIdLoopBus,
                                       &loopFilteredL, 1);

            leftOut += loopFilteredL;
            rightOut += loopFilteredR;

            if (delayTargetsLoop &&
                (loopDelaySumL != 0.0F || loopDelaySumR != 0.0F)) {
                float delayL = loopDelaySumL;
                float delayR = loopDelaySumR;
                processDelayBusSample(delayL, delayR);
                if (delayBusActive) {
                    delayHadSourceInputThisSample = true;
                }
                leftOut += delayL;
                rightOut += delayR;
            }

            if (reverbTargetsLoop &&
                (loopReverbSumL != 0.0F || loopReverbSumR != 0.0F)) {
                float reverbL = loopReverbSumL;
                float reverbR = loopReverbSumR;
                processReverbBusSample(reverbL, reverbR);
                if (reverbBusActive) {
                    reverbHadSourceInputThisSample = true;
                }
                leftOut += reverbL;
                rightOut += reverbR;
            }

            // Apply global Delay / Reverb FX when configured.
            const float globalFxInputL = leftOut;
            const float globalFxInputR = rightOut;

            if (delayApplyGlobal) {
                float delayL = globalFxInputL;
                float delayR = globalFxInputR;
                processDelayBusSample(delayL, delayR);
                if (delayBusActive) {
                    delayHadSourceInputThisSample = true;
                }
                leftOut += delayL;
                rightOut += delayR;
            }

            if (reverbApplyGlobal) {
                float reverbL = globalFxInputL;
                float reverbR = globalFxInputR;
                processReverbBusSample(reverbL, reverbR);
                if (reverbBusActive) {
                    reverbHadSourceInputThisSample = true;
                }
                leftOut += reverbL;
                rightOut += reverbR;
            }

            if (!delayHadSourceInputThisSample && delayBusActive &&
                delayTailSamples > 0) {
                float tailL = 0.0F;
                float tailR = 0.0F;
                processDelayBusSample(tailL, tailR);
                leftOut += tailL;
                rightOut += tailR;
            }

            if (!reverbHadSourceInputThisSample && reverbBusActive &&
                reverbTailSamples > 0) {
                float tailL = 0.0F;
                float tailR = 0.0F;
                processReverbBusSample(tailL, tailR);
                leftOut += tailL;
                rightOut += tailR;
            }

            if (delayHadSourceInputThisSample) {
                delayTailSamples = kDelayTailHoldSamples;
            } else if (delayTailSamples > 0) {
                --delayTailSamples;
            }

            if (reverbHadSourceInputThisSample) {
                reverbTailSamples = kDelayTailHoldSamples;
            } else if (reverbTailSamples > 0) {
                --reverbTailSamples;
            }

            leftOut = juce::jlimit(-0.9F, 0.9F, leftOut);
            rightOut = juce::jlimit(-0.9F, 0.9F, rightOut);

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

    delayTailSamplesRemaining_.store(delayTailSamples,
                                     std::memory_order_relaxed);
    reverbTailSamplesRemaining_.store(reverbTailSamples,
                                      std::memory_order_relaxed);

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

    // Determine the effective beat count for this sample. Priority:
    //   1) If the caller provides a positive `beats` value (e.g. from
    //      .rtp metadata), trust it and cache it per file path.
    //   2) If `beats <= 0` but we have a cached value for this audio
    //      file, reuse the cached beats so that samples keep their
    //      original length even when selected from different Loop
    //      modules.
    //   3) Otherwise, estimate beats from the decoded duration and
    //      current loop BPM, biasing towards powers of two where
    //      possible and caching the result.

    int effectiveBeats = beats;

    if (effectiveBeats > 0) {
        loopSampleBeatsCache_[canonicalPath] = effectiveBeats;
    } else {
        // Try to reuse any previously known beat count for this
        // sample derived either from .rtp metadata or a prior
        // auto-detection.
        const auto beatsIt =
            loopSampleBeatsCache_.find(canonicalPath);
        if (beatsIt != loopSampleBeatsCache_.end() &&
            beatsIt->second > 0) {
            effectiveBeats = beatsIt->second;
        } else if (sample.buffer != nullptr &&
                   sample.buffer->numFrames > 0 &&
                   sample.buffer->sourceSampleRate > 0.0) {
            const double durationSeconds =
                static_cast<double>(sample.buffer->numFrames) /
                sample.buffer->sourceSampleRate;
            const float bpm =
                loopGlobalBpm_.load(std::memory_order_relaxed);

            if (durationSeconds > 0.0 && bpm > 0.0F) {
                const double rawBeats =
                    durationSeconds * static_cast<double>(bpm) /
                    60.0;

                // Clamp to a musically sensible range to avoid
                // exploding tempo factors in pathological cases.
                constexpr int kMinBeats = 1;
                constexpr int kMaxBeats = 64;

                int estimated = 0;

                // First, try snapping to the nearest power of two
                // within a reasonable relative error. This favours
                // common musical lengths such as 1, 2, 4, 8, 16
                // beats even when the session BPM does not match the
                // original loop tempo exactly.
                const int candidatePowers[] = {1, 2, 4, 8, 16, 32, 64};
                double bestError = std::numeric_limits<double>::max();
                int bestCandidate = 0;

                for (int candidate : candidatePowers) {
                    const double err = std::abs(
                        static_cast<double>(candidate) - rawBeats);
                    const double rel = (candidate > 0)
                                           ? (err / candidate)
                                           : err;
                    // Require the candidate to be within 30% of the
                    // measured value to avoid snapping wildly.
                    if (rel <= 0.30 && err < bestError) {
                        bestError = err;
                        bestCandidate = candidate;
                    }
                }

                if (bestCandidate > 0) {
                    estimated = bestCandidate;
                } else {
                    // Fallback: nearest integer beat count.
                    const int rounded = static_cast<int>(
                        std::round(rawBeats));
                    if (rounded >= kMinBeats &&
                        rounded <= kMaxBeats) {
                        estimated = rounded;
                    }
                }

                if (estimated >= kMinBeats &&
                    estimated <= kMaxBeats) {
                    effectiveBeats = estimated;
                    loopSampleBeatsCache_[canonicalPath] =
                        effectiveBeats;
                }
            }
        }
    }

    if (effectiveBeats < 0) {
        effectiveBeats = 0;
    }

    sample.beats = effectiveBeats;

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

    juce::Logger::writeToLog(
        juce::String("[rectai-core] Loop: loaded sample for module=") +
        juce::String(moduleId) + " slot=" + juce::String(slotIndex) +
        " frames=" +
        juce::String(instance.slots[static_cast<std::size_t>(slotIndex)]
                         .buffer->numFrames) +
        " beats=" +
        juce::String(instance.slots[static_cast<std::size_t>(slotIndex)]
                         .beats));

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
                                      const float releaseMs,
                                      const bool routeThroughFilter)
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

    instance.routeThroughFilter.store(routeThroughFilter,
                                      std::memory_order_relaxed);

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
            snapIt->second.routeThroughFilter.store(
                routeThroughFilter, std::memory_order_relaxed);
        }
    }
}

void AudioEngine::setLoopGlobalTempo(const float bpm)
{
    const float clamped = bpm > 0.0F ? bpm : 0.0F;
    loopGlobalBpm_.store(clamped, std::memory_order_relaxed);
}

void AudioEngine::setDelayFxParams(const int mode,
                                   const float delayNormalised,
                                   const float feedback,
                                   const float reverbAmount,
                                   const float wetGain,
                                   DelayTargetConfig target)
{
    const int clampedMode = juce::jlimit(0, 2, mode);

    auto applyRouting = [&](auto& busState) {
        busState.applyGlobal.store(target.applyToGlobalMix ? 1 : 0,
                                   std::memory_order_relaxed);
        busState.includeSampleplay.store(
            target.includeSampleplayBus ? 1 : 0,
            std::memory_order_relaxed);
        busState.voiceMask.store(target.voiceMask,
                                 std::memory_order_relaxed);

        const int clampedLoopCount = juce::jlimit(
            0, DelayTargetConfig::kMaxLoopTargets, target.loopTargetCount);
        busState.loopTargetCount.store(clampedLoopCount,
                                       std::memory_order_relaxed);
        for (int i = 0; i < DelayTargetConfig::kMaxLoopTargets; ++i) {
            const std::uint64_t hash =
                (i < clampedLoopCount)
                    ? target.loopHashes[static_cast<std::size_t>(i)]
                    : 0ULL;
            busState.loopTargetHashes[static_cast<std::size_t>(i)].store(
                hash, std::memory_order_relaxed);
        }

        const bool hasTargets = target.applyToGlobalMix ||
                                 target.includeSampleplayBus ||
                                 target.voiceMask != 0u ||
                                 clampedLoopCount > 0;
        const bool busActive = hasTargets || target.keepFxAlive;
        busState.active.store(busActive ? 1 : 0,
                              std::memory_order_relaxed);
        return busActive;
    };

    switch (clampedMode) {
        case 0: {
            delayBus_.active.store(0, std::memory_order_relaxed);
            reverbBus_.active.store(0, std::memory_order_relaxed);
            delayTailSamplesRemaining_.store(0, std::memory_order_relaxed);
            reverbTailSamplesRemaining_.store(0, std::memory_order_relaxed);
            break;
        }
        case 1: {
            const float dNorm =
                juce::jlimit(0.0F, 1.0F, delayNormalised);
            const float fbClamped =
                juce::jlimit(0.0F, 0.95F, feedback);
            const float wetClamped =
                juce::jlimit(0.0F, 1.0F, wetGain);

            delayBus_.delayNormalised.store(dNorm,
                                            std::memory_order_relaxed);
            delayBus_.feedback.store(fbClamped,
                                     std::memory_order_relaxed);
            delayBus_.wetGain.store(wetClamped,
                                    std::memory_order_relaxed);

            const bool active = applyRouting(delayBus_);
            if (!active) {
                delayTailSamplesRemaining_.store(
                    0, std::memory_order_relaxed);
            }
            break;
        }
        case 2: {
            const float revClamped =
                juce::jlimit(0.0F, 1.0F, reverbAmount);
            const float wetClamped =
                juce::jlimit(0.0F, 1.0F, wetGain);

            reverbBus_.reverbAmount.store(revClamped,
                                          std::memory_order_relaxed);
            reverbBus_.wetGain.store(wetClamped,
                                     std::memory_order_relaxed);

            const bool active = applyRouting(reverbBus_);
            if (!active) {
                reverbTailSamplesRemaining_.store(
                    0, std::memory_order_relaxed);
            }
            break;
        }
        default:
            break;
    }
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

int AudioEngine::getLoopSampleBeats(const std::string& moduleId,
                                    const int slotIndex) const
{
    if (slotIndex < 0) {
        return 0;
    }

    const auto it = loopModules_.find(moduleId);
    if (it == loopModules_.end()) {
        return 0;
    }

    const LoopInstance& instance = it->second;
    if (slotIndex >= static_cast<int>(instance.slots.size())) {
        return 0;
    }

    const LoopSample& sample =
        instance.slots[static_cast<std::size_t>(slotIndex)];
    return sample.beats;
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
    setBusFilter(kBusFilterSampleplay, mode, cutoffHz, q);
}

void AudioEngine::setLoopFilter(const int mode,
                                const double cutoffHz,
                                const float q)
{
    setBusFilter(kBusFilterLoop, mode, cutoffHz, q);
}

void AudioEngine::setBusFilter(const int busIndex, const int mode,
                               const double cutoffHz, const float q)
{
    if (busIndex < 0 || busIndex >= kNumBusFilters) {
        return;
    }

    auto& bus = busFilters_[busIndex];

    // FilterModule modes are defined as:
    // 0 = low-pass, 1 = band-pass, 2 = high-pass. We mirror this
    // mapping directly on the audio side so that UI mode ids can be
    // passed through unchanged. Mode is not used as an explicit
    // bypass flag here; instead, a cutoff <= 0 disables processing.
    const int clampedMode = juce::jlimit(0, 2, mode);

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

    bus.mode.store(clampedMode, std::memory_order_relaxed);
    bus.cutoffHz.store(fc, std::memory_order_relaxed);
    bus.q.store(qClamped, std::memory_order_relaxed);

    // If the cutoff is invalid or zero, leave the filter effectively
    // bypassed. We avoid resetting here to preserve continuity when
    // parameters are toggled.
    if (fc <= 0.0) {
        return;
    }

    using FilterType = juce::dsp::StateVariableTPTFilterType;

    switch (clampedMode) {
        case 0:  // Low-pass
            bus.filterL.setType(FilterType::lowpass);
            bus.filterR.setType(FilterType::lowpass);
            break;
        case 1:  // Band-pass
            bus.filterL.setType(FilterType::bandpass);
            bus.filterR.setType(FilterType::bandpass);
            break;
        case 2:  // High-pass
            bus.filterL.setType(FilterType::highpass);
            bus.filterR.setType(FilterType::highpass);
            break;
        default:
            return;
    }

    bus.filterL.setCutoffFrequency(static_cast<float>(fc));
    bus.filterR.setCutoffFrequency(static_cast<float>(fc));
    bus.filterL.setResonance(qClamped);
    bus.filterR.setResonance(qClamped);
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

void AudioEngine::getSampleplayBusWaveformSnapshot(float* dst,
                                                   const int numPoints,
                                                   const double windowSeconds)
{
    if (dst == nullptr || numPoints <= 0 || sampleRate_ <= 0.0) {
        return;
    }

    // Delegate to the unified visual Voices container using the
    // dedicated Sampleplay mono voice id. This keeps the UI path
    // independent from the legacy connection tap buffers.
    visualVoices_.getSnapshot(kVisualVoiceIdSampleplayRaw, dst,
                              numPoints, windowSeconds);
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
    // Loop per-module waveforms are now sourced from the unified
    // visual Voices container rather than the legacy
    // `loopWaveformBuffers_`. Each LoopModule with a visual slot
    // reserves a voice id in the range
    // [kMaxVoices, kMaxVoices + kMaxLoopWaveforms).
    const int voiceId = kMaxVoices + wfIndex;
    visualVoices_.getSnapshot(voiceId, dst, numPoints, windowSeconds);
}

void AudioEngine::getLoopBusWaveformSnapshot(float* dst,
                                             const int numPoints,
                                             const double windowSeconds)
{
    if (dst == nullptr || numPoints <= 0 || sampleRate_ <= 0.0) {
        return;
    }

    visualVoices_.getSnapshot(kVisualVoiceIdLoopBus, dst, numPoints,
                              windowSeconds);
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

    // FilterModule modes are defined as 0 = low-pass, 1 = band-pass,
    // 2 = high-pass. Mirror this mapping directly for per-voice
    // filters so that UI mode ids can be forwarded unchanged. As
    // with bus filters, bypass is handled via cutoff <= 0 rather
    // than a dedicated mode value.
    const int clampedMode = juce::jlimit(0, 2, mode);

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

    // If the cutoff is invalid or zero, leave the filter effectively
    // bypassed. We avoid resetting here to preserve continuity when
    // parameters are toggled.
    if (fc <= 0.0) {
        return;
    }

    switch (clampedMode) {
        case 0: // Low-pass
            filters_[index].setType(
                juce::dsp::StateVariableTPTFilterType::lowpass);
            break;
        case 1: // Band-pass
            filters_[index].setType(
                juce::dsp::StateVariableTPTFilterType::bandpass);
            break;
        case 2: // High-pass
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
    // Delegate to the dedicated pre-filter oscillator history so that
    // callers can visualise the generator signal before the
    // per-voice filter is applied (e.g. Osc → Filter connections).
    oscPreVoices_.getSnapshot(voiceIndex, dst, numPoints,
                              windowSeconds);
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
    // For filtered per-voice visuals we also read from the unified
    // Voices container, which stores the post-filter oscillator
    // signal written in the audio callback for ids [0, kMaxVoices).
    visualVoices_.getSnapshot(voiceIndex, dst, numPoints, windowSeconds);
}
