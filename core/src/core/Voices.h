#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace soundtable {

// Lightweight, JUCE-free helper that owns circular mono buffers for
// visual "voices". Each voice represents the audio output of a
// module (generator or processor) from the perspective of the UI.
//
// Design goals:
//   - Single writer (audio thread), multiple readers (UI thread).
//   - Lock-free: audio thread never locks; readers use a stable
//     snapshot of the write index.
//   - Time-aligned history window per voice so that the UI can draw
//     per-module waveforms over a short temporal window (≈50 ms).
//
// Voices does not know about Scene, AudioGraph or module ids. Higher
// layers are responsible for mapping logical modules to VoiceId
// instances and for forwarding the relevant mono samples from the
// audio pipeline.
class Voices {
public:
    using VoiceId = int;

    // Constructs a Voices container with the given limits. The
    // history size is specified in mono samples per voice.
    Voices(int maxVoices, int historySize) noexcept;

    Voices(const Voices&) = delete;
    Voices& operator=(const Voices&) = delete;

    Voices(Voices&&) = delete;
    Voices& operator=(Voices&&) = delete;

    ~Voices() = default;

    // Number of logical voices managed by this instance.
    [[nodiscard]] int maxVoices() const noexcept { return maxVoices_; }

    // History length per voice in mono samples.
    [[nodiscard]] int historySize() const noexcept { return historySize_; }

    // Sets the sample rate used to interpret `windowSeconds` in
    // getSnapshot. This should be called from the audio thread when
    // the device starts or its configuration changes.
    void setSampleRate(double sampleRate) noexcept;

    // Resets all history buffers to silence and clears the write
    // index. This is typically called from the audio thread when the
    // engine (re)starts.
    void reset() noexcept;

    // Writes a contiguous block of mono samples for the given voice
    // into its circular history buffer. The caller is responsible for
    // ensuring that `voice` is in [0, maxVoices()). The audio thread
    // is expected to call this once per processing block per active
    // voice.
    void writeSamples(VoiceId voice, const float* mono, int numSamples) noexcept;

    // Copies a downsampled snapshot of the recent mono history for
    // the given voice into `dst`. The snapshot covers approximately
    // `windowSeconds` of audio (clamped to the available history) and
    // is safe to call from the UI thread without blocking the audio
    // callback. When there is not enough history yet or the voice is
    // out of range, `dst` is filled with zeros.
    void getSnapshot(VoiceId voice,
                     float* dst,
                     int numPoints,
                     double windowSeconds) const noexcept;

private:
    const int maxVoices_;
    const int historySize_;

    // Per-voice write indices in samples. Each voice maintains its own
    // circular history so that callers can write samples incrementally
    // (for example, one sample at a time from the audio callback)
    // without creating gaps in the per-voice timeline.
    std::vector<std::atomic<int>> writeIndices_;

    std::atomic<double> sampleRate_{44100.0};

    // Flattened storage: [voice][sample] laid out as a single
    // vector. Access pattern is:
    //   buffer_[voice * historySize_ + localIndex]
    std::vector<float> buffer_;
};

}  // namespace soundtable
