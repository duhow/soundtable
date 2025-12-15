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
    // Reset phases for all voices.
    for (double& phase : phases_) {
        phase = 0.0;
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

        if (voiceCount > 0) {
            for (int v = 0; v < voiceCount; ++v) {
                const double freq =
                    voices_[v].frequency.load(std::memory_order_relaxed);
                const float level =
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

                // Store both pre- and post-filter waveforms for
                // visualisation so that connections from generators
                // (Osc → Filter) can show the raw oscillator shape
                // while filters/FX and master visuals can reflect
                // the processed signal.
                voicePreFilterWaveformBuffer_[v][bufIndex] = raw;
                voicePostFilterWaveformBuffer_[v][bufIndex] = s;

                // Update any connection taps that monitor this
                // voice. Taps can either observe the pre-filter
                // oscillator signal or the post-filter processed
                // signal.
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

                    if (tapKind ==
                        static_cast<int>(ConnectionTapSourceKind::kVoicePre)) {
                        connectionWaveformBuffers_[t][bufIndex] = raw;
                    } else if (tapKind ==
                               static_cast<int>(
                                   ConnectionTapSourceKind::kVoicePost)) {
                        connectionWaveformBuffers_[t][bufIndex] = s;
                    }
                }
                
                // Apply level scaling for audio output.
                const float scaledOutput = s * level;
                oscMixed += scaledOutput;
            }

            // Prevent hard digital clipping when summing several
            // voices or using high-Q filter settings on the
            // oscillator side. FluidSynth output is assumed to be
            // already well behaved; we clip after mixing both.
            oscMixed = juce::jlimit(-0.9F, 0.9F, oscMixed);

            // Store Sampleplay-only contributions for visualisation:
            // - sampleplayWaveformBuffer_: original (pre-filter)
            // - sampleplayFilteredWaveformBuffer_: processed
            const float sampleplayMonoRaw = rawSampleplayL;
            const float sampleplayMonoFiltered = filteredSampleplayL;
            sampleplayWaveformBuffer_[bufIndex] = sampleplayMonoRaw;
            sampleplayFilteredWaveformBuffer_[bufIndex] =
                sampleplayMonoFiltered;

            // Update any connection taps attached to the Sampleplay
            // path using the **original** (pre-filter) mono
            // waveform. Esto permite que la conexión Sampleplay →
            // Filter dibuje siempre la waveform "original" del
            // SoundFont, incluso cuando el filtro global de
            // Sampleplay está activo.
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

            const float leftOut = juce::jlimit(
                -0.9F, 0.9F,
                filteredSampleplayL + oscMixed);
            const float rightOut = juce::jlimit(
                -0.9F, 0.9F,
                filteredSampleplayR + oscMixed);

            // Store the mixed mono signal (left channel) for
            // waveform visualisation.
            waveformBuffer_[bufIndex] = leftOut;

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
            const float leftOut = juce::jlimit(
                -0.9F, 0.9F,
                sampleplayLeft_[static_cast<std::size_t>(sample)]);
            const float rightOut = juce::jlimit(
                -0.9F, 0.9F,
                sampleplayRight_[static_cast<std::size_t>(sample)]);

            waveformBuffer_[bufIndex] = leftOut;
            const float sampleplayMonoRaw = rawSampleplayL;
            const float sampleplayMonoFiltered = filteredSampleplayL;
            sampleplayWaveformBuffer_[bufIndex] =
                sampleplayMonoRaw;
            sampleplayFilteredWaveformBuffer_[bufIndex] =
                sampleplayMonoFiltered;

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

void AudioEngine::getSampleplayWaveformSnapshot(float* dst,
                                                const int numPoints,
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

    const int points = std::min(numPoints, windowSamples);
    const float denom = static_cast<float>(std::max(points - 1, 1));

    for (int i = 0; i < points; ++i) {
        const float t = static_cast<float>(i) / denom;
        const int offset = static_cast<int>(t * static_cast<float>(windowSamples - 1));
        const int bufIndex =
            (startIndex + offset + historySize) % historySize;
        dst[i] = sampleplayWaveformBuffer_[bufIndex];
    }

    for (int i = points; i < numPoints; ++i) {
        dst[i] = dst[points - 1];
    }
}

void AudioEngine::getSampleplayFilteredWaveformSnapshot(
    float* dst, const int numPoints, const double windowSeconds)
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
        dst[i] = sampleplayFilteredWaveformBuffer_[bufIndex];
    }

    for (int i = points; i < numPoints; ++i) {
        dst[i] = dst[points - 1];
    }
}
