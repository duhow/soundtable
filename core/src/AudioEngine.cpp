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
}

void AudioEngine::audioDeviceStopped()
{
    for (double& phase : phases_) {
        phase = 0.0;
    }
}

void AudioEngine::setFrequency(const double frequency)
{
    // Convenience wrapper for voice 0.
    setVoice(0, frequency, voices_[0].level.load(std::memory_order_relaxed));
}

void AudioEngine::setLevel(const float level)
{
    // Convenience wrapper for voice 0.
    setVoice(0, voices_[0].frequency.load(std::memory_order_relaxed), level);
}

void AudioEngine::setVoice(const int index, const double frequency,
                           const float level)
{
    if (index < 0 || index >= kMaxVoices) {
        return;
    }

    const auto clampedFreq = juce::jlimit(20.0, 2000.0, frequency);
    const auto clampedLevel = juce::jlimit(0.0F, 1.0F, level);

    voices_[index].frequency.store(clampedFreq, std::memory_order_relaxed);
    voices_[index].level.store(clampedLevel, std::memory_order_relaxed);

    int currentVoices = numVoices_.load(std::memory_order_relaxed);
    const int desired = index + 1;
    while (desired > currentVoices &&
           !numVoices_.compare_exchange_weak(currentVoices, desired,
                                             std::memory_order_relaxed)) {
        // currentVoices is updated with the latest value; loop until we
        // successfully raise it or observe a newer, larger value.
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
    if (numSamples <= 0 || numOutputChannels <= 0) {
        return;
    }

    // Clear all output channels first.
    for (int channel = 0; channel < numOutputChannels; ++channel) {
        if (auto* buffer = outputChannelData[channel]) {
            std::fill(buffer, buffer + numSamples, 0.0F);
        }
    }

    // Generate a simple multi-voice sine mix and write it to all outputs.
    const int voiceCount =
        std::clamp(numVoices_.load(std::memory_order_relaxed), 0, kMaxVoices);

    for (int sample = 0; sample < numSamples; ++sample) {
        float mixed = 0.0F;

        if (sampleRate_ > 0.0 && voiceCount > 0) {
            const double twoPiOverFs =
                juce::MathConstants<double>::twoPi / sampleRate_;

            for (int v = 0; v < voiceCount; ++v) {
                const auto freq =
                    voices_[v].frequency.load(std::memory_order_relaxed);
                const auto level =
                    voices_[v].level.load(std::memory_order_relaxed);

                if (level <= 0.0F || freq <= 0.0) {
                    continue;
                }

                phases_[v] += twoPiOverFs * freq;
                const auto s = static_cast<float>(std::sin(phases_[v])) *
                               level;
                mixed += s;
            }
        }

        for (int channel = 0; channel < numOutputChannels; ++channel) {
            if (auto* buffer = outputChannelData[channel]) {
                buffer[sample] = mixed;
            }
        }
    }
}
