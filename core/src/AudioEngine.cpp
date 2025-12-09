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

    const double frequency = targetFrequency_.load(std::memory_order_relaxed);
    angleDelta_ = juce::MathConstants<double>::twoPi * frequency / sampleRate_;
    currentAngle_ = 0.0;
}

void AudioEngine::audioDeviceStopped()
{
    currentAngle_ = 0.0;
    angleDelta_ = 0.0;
}

void AudioEngine::setFrequency(const double frequency)
{
    // Clamp to a reasonable range to avoid extreme values.
    const auto clamped = juce::jlimit(20.0, 2000.0, frequency);
    targetFrequency_.store(clamped, std::memory_order_relaxed);
}

void AudioEngine::setLevel(const float level)
{
    const auto clamped = juce::jlimit(0.0F, 1.0F, level);
    level_.store(clamped, std::memory_order_relaxed);
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

    // Update angle delta according to the latest target frequency.
    const auto frequency = targetFrequency_.load(std::memory_order_relaxed);
    if (sampleRate_ > 0.0) {
        angleDelta_ =
            juce::MathConstants<double>::twoPi * frequency / sampleRate_;
    }

    // Clear all output channels first.
    for (int channel = 0; channel < numOutputChannels; ++channel) {
        if (auto* buffer = outputChannelData[channel]) {
            std::fill(buffer, buffer + numSamples, 0.0F);
        }
    }

    const auto level = level_.load(std::memory_order_relaxed);

    // Generate a simple sine wave and write it to all output channels.
    for (int sample = 0; sample < numSamples; ++sample) {
        const auto currentSample =
            static_cast<float>(std::sin(currentAngle_)) * level;
        currentAngle_ += angleDelta_;

        for (int channel = 0; channel < numOutputChannels; ++channel) {
            if (auto* buffer = outputChannelData[channel]) {
                buffer[sample] = currentSample;
            }
        }
    }
}
