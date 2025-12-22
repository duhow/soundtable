#include "core/Voices.h"

#include <algorithm>

namespace rectai {

Voices::Voices(const int maxVoices, const int historySize) noexcept
    : maxVoices_(maxVoices > 0 ? maxVoices : 0),
      historySize_(historySize > 0 ? historySize : 0),
      writeIndices_(static_cast<std::size_t>(
          maxVoices_ > 0 ? maxVoices_ : 0)),
      buffer_(static_cast<std::size_t>(
          (maxVoices_ > 0 && historySize_ > 0)
              ? (maxVoices_ * historySize_)
              : 0),
              0.0F)
{
}

void Voices::setSampleRate(const double sampleRate) noexcept
{
    const double sr = (sampleRate > 0.0) ? sampleRate : 44100.0;
    sampleRate_.store(sr, std::memory_order_relaxed);
}

void Voices::reset() noexcept
{
    for (int v = 0; v < maxVoices_; ++v) {
        writeIndices_[static_cast<std::size_t>(v)].store(
            0, std::memory_order_relaxed);
    }
    std::fill(buffer_.begin(), buffer_.end(), 0.0F);
}

void Voices::writeSamples(const VoiceId voice,
                          const float* mono,
                          const int numSamples) noexcept
{
    if (mono == nullptr || numSamples <= 0 ||
        voice < 0 || voice >= maxVoices_ ||
        historySize_ <= 0 || buffer_.empty()) {
        return;
    }

    const int historySize = historySize_;

    const std::size_t voiceIdx =
        static_cast<std::size_t>(voice);
    const int baseWrite = writeIndices_[voiceIdx].fetch_add(
        numSamples, std::memory_order_relaxed);

    const std::size_t voiceOffset =
        static_cast<std::size_t>(voice) *
        static_cast<std::size_t>(historySize);

    for (int i = 0; i < numSamples; ++i) {
        const int globalIndex = baseWrite + i;
        const int localIndex =
            historySize > 0 ? (globalIndex % historySize) : 0;
        const std::size_t idx = voiceOffset +
                                static_cast<std::size_t>(localIndex);
        buffer_[idx] = mono[i];
    }
}

void Voices::getSnapshot(const VoiceId voice,
                         float* dst,
                         const int numPoints,
                         double windowSeconds) const noexcept
{
    if (dst == nullptr || numPoints <= 0 ||
        voice < 0 || voice >= maxVoices_ ||
        historySize_ <= 0 || buffer_.empty()) {
        return;
    }

    const int historySize = historySize_;
    const std::size_t voiceIdx =
        static_cast<std::size_t>(voice);
    const int writeIdx = writeIndices_[voiceIdx].load(
        std::memory_order_relaxed);
    const int availableSamples =
        std::min(historySize, std::max(writeIdx, 0));

    if (availableSamples <= 0) {
        std::fill(dst, dst + numPoints, 0.0F);
        return;
    }

    double window = windowSeconds;
    if (window <= 0.0) {
        window = 0.05;  // ~50 ms default.
    }

    const double sr = sampleRate_.load(std::memory_order_relaxed);
    const double srSafe = (sr > 0.0) ? sr : 44100.0;

    int windowSamples = static_cast<int>(window * srSafe);
    windowSamples = std::max(1, std::min(windowSamples, availableSamples));

    const int startIndex =
        (writeIdx - windowSamples + historySize * 4) % historySize;

    const int points = std::min(numPoints, windowSamples);
    const float denom = static_cast<float>(
        std::max(points - 1, 1));

    const std::size_t voiceOffset =
        static_cast<std::size_t>(voice) *
        static_cast<std::size_t>(historySize);

    for (int i = 0; i < points; ++i) {
        const float t = static_cast<float>(i) / denom;
        const int offset = static_cast<int>(
            t * static_cast<float>(windowSamples - 1));
        const int localIndex =
            (startIndex + offset + historySize) % historySize;
        const std::size_t idx = voiceOffset +
                                static_cast<std::size_t>(localIndex);
        dst[i] = buffer_[idx];
    }

    // Pad with the last value when the requested resolution is
    // higher than the available history.
    for (int i = points; i < numPoints; ++i) {
        dst[i] = dst[points - 1];
    }
}

}  // namespace rectai
