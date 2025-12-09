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

    const double frequency = 440.0;  // A4 test tone.
    angleDelta_ = juce::MathConstants<double>::twoPi * frequency / sampleRate_;
    currentAngle_ = 0.0;
}

void AudioEngine::audioDeviceStopped()
{
    currentAngle_ = 0.0;
    angleDelta_ = 0.0;
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

    // Generate a simple sine wave and write it to all output channels.
    for (int sample = 0; sample < numSamples; ++sample) {
        const auto currentSample =
            static_cast<float>(std::sin(currentAngle_)) * level_;
        currentAngle_ += angleDelta_;

        for (int channel = 0; channel < numOutputChannels; ++channel) {
            if (auto* buffer = outputChannelData[channel]) {
                buffer[sample] = currentSample;
            }
        }
    }
}
