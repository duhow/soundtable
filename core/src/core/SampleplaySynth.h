#pragma once

#include <mutex>
#include <string>

// Lightweight wrapper around FluidSynth used to render SoundFont2
// instruments for Sampleplay modules. This class is deliberately kept
// independent of JUCE so it can live inside rectai-core-lib. All
// methods are thread-safe via an internal mutex, but note that calling
// them from a real-time audio thread may introduce contention; this is
// acceptable for the current prototype.

namespace rectai {

class SampleplaySynth {
 public:
  SampleplaySynth();
  ~SampleplaySynth();

  // Sets the desired sample rate for subsequent rendering. Changing the
  // sample rate recreates the internal FluidSynth objects; any loaded
  // soundfont will need to be reloaded afterwards via loadSoundfont().
  void setSampleRate(double sampleRate);

  // Loads the given SoundFont2 file into FluidSynth. On success,
  // returns true and clears error_message (if non-null). On failure,
  // returns false and writes a short description into error_message.
  bool loadSoundfont(const std::string& path,
                     std::string* error_message = nullptr);

  // Triggers a note using the given bank/program pair (MIDI-style
  // preset selection), MIDI key (0-127) and normalised velocity in
  // [0,1]. If no soundfont is loaded, the call is ignored.
  void noteOn(int bank, int program, int midiKey, float velocity01);

  // Sends a note-off for the given MIDI key on the default channel.
  // This is currently unused by the caller, as most Reactable
  // Sampleplay instruments are short percussive sounds, but it is
  // provided for future extensions.
  void noteOff(int midiKey);

  // Renders numSamples of stereo audio into the provided left/right
  // buffers. If no soundfont is loaded the buffers are filled with
  // silence.
  void render(float* left, float* right, int numSamples);

 private:
  void clearSynthLocked();
  void ensureSynthLocked();

  mutable std::mutex mutex_;

  struct Impl;
  Impl* impl_{nullptr};
};

}  // namespace rectai
