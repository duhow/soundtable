#include "AudioEngine.h"

#include <algorithm>
#include <cmath>

AudioEngine::AudioEngine()
{
    // Initialise with no inputs and stereo outputs.
    juce::String audioError =
        deviceManager_.initialiseWithDefaultDevices(/*numInputChannels*/ 0,
                                                    /*numOutputChannels*/ 2);
    if (audioError.isNotEmpty()) {
        juce::Logger::writeToLog("[rectai-core] Failed to initialise audio: " +
                                 audioError);
    } else {
        deviceManager_.addAudioCallback(this);
        juce::Logger::writeToLog("[rectai-core] Audio engine initialised.");
    }
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

    // Reserve a contiguous block of indices in the circular history
    // buffer for this callback.
    const int baseWriteIndex =
        waveformWriteIndex_.fetch_add(numSamples, std::memory_order_relaxed);

    for (int sample = 0; sample < numSamples; ++sample) {
        float mixed = 0.0F;

        const int writeIndex = baseWriteIndex + sample;
        const int bufIndex =
            historySize > 0 ? (writeIndex % historySize) : 0;

        if (voiceCount > 0) {
            for (int v = 0; v < voiceCount; ++v) {
                const double freq =
                    voices_[v].frequency.load(std::memory_order_relaxed);
                const float level =
                    voices_[v].level.load(std::memory_order_relaxed);

                float raw = 0.0F;
                if (level > 0.0F && freq > 0.0) {
                    phases_[v] += twoPiOverFs * freq;
                    raw = static_cast<float>(std::sin(phases_[v])) *
                          level;
                }

                // Apply optional per-voice filter via JUCE's
                // StateVariableTPTFilter for better stability.
                float s = raw;
                const int mode =
                    voices_[v].filterMode.load(std::memory_order_relaxed);
                if (mode != 0) {
                    s = filters_[v].processSample(0, raw);
                }

                mixed += s;
                voiceWaveformBuffer_[v][bufIndex] = s;
            }

            // Prevent hard digital clipping when summing several
            // voices or using high-Q filter settings.
            mixed = juce::jlimit(-0.9F, 0.9F, mixed);
            waveformBuffer_[bufIndex] = mixed;
        } else {
            waveformBuffer_[bufIndex] = 0.0F;
            for (int v = 0; v < kMaxVoices; ++v) {
                voiceWaveformBuffer_[v][bufIndex] = 0.0F;
            }
        }

        for (int channel = 0; channel < numOutputChannels; ++channel) {
            if (auto* buffer = outputChannelData[channel]) {
                buffer[sample] = mixed;
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

void AudioEngine::setVoiceFilter(const int index, const int mode,
                                 const double cutoffHz, const float q)
{
    if (index < 0 || index >= kMaxVoices) {
        return;
    }

    const int clampedMode = juce::jlimit(0, 3, mode);
    voices_[index].filterMode.store(clampedMode, std::memory_order_relaxed);
    voices_[index].filterCutoffHz.store(cutoffHz,
                                        std::memory_order_relaxed);

    // Resonance must be strictly > 0 for the JUCE filter; clamp
    // gently to a musically useful range to avoid extreme peaks.
    const float qClamped = juce::jlimit(0.1F, 8.0F, q);
    voices_[index].filterQ.store(qClamped, std::memory_order_relaxed);

    filters_[index].reset();

    if (clampedMode == 0) {
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

    const double sr = sampleRate_ > 0.0 ? sampleRate_ : 44100.0;
    const double nyquist = 0.5 * sr;
    double fc = cutoffHz;
    if (fc <= 0.0) {
        fc = 0.0;
    }
    if (fc >= nyquist) {
        // JUCE exige cutoff < Nyquist, no <=.
        fc = nyquist * 0.99;
    }

    if (fc <= 0.0) {
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
        const int offset = static_cast<int>(t * static_cast<float>(windowSamples - 1));
        const int bufIndex =
            (startIndex + offset + historySize) % historySize;
        dst[i] = voiceWaveformBuffer_[voiceIndex][bufIndex];
    }

    for (int i = points; i < numPoints; ++i) {
        dst[i] = dst[points - 1];
    }
}
