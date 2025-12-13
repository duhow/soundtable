#include "core/SampleplaySynth.h"

#include <algorithm>

#include <fluidsynth.h>

namespace rectai {

// Small pImpl to keep FluidSynth types out of the public header and
// avoid leaking implementation details.
struct SampleplaySynth::Impl {
  fluid_settings_t* settings{nullptr};
  fluid_synth_t* synth{nullptr};
  int sfontId{-1};
  double sampleRate{44100.0};
  std::string currentPath;
};

SampleplaySynth::SampleplaySynth() : impl_(new Impl) {}

SampleplaySynth::~SampleplaySynth()
{
  std::lock_guard<std::mutex> lock(mutex_);
  clearSynthLocked();
  delete impl_;
  impl_ = nullptr;
}

void SampleplaySynth::clearSynthLocked()
{
  if (impl_ == nullptr) {
    return;
  }

  if (impl_->synth != nullptr) {
    if (impl_->sfontId >= 0) {
      (void)fluid_synth_sfunload(impl_->synth, impl_->sfontId, 1);
      impl_->sfontId = -1;
    }
    delete_fluid_synth(impl_->synth);
    impl_->synth = nullptr;
  }

  if (impl_->settings != nullptr) {
    delete_fluid_settings(impl_->settings);
    impl_->settings = nullptr;
  }

  impl_->currentPath.clear();
}

void SampleplaySynth::ensureSynthLocked()
{
  if (impl_ == nullptr) {
    return;
  }

  if (impl_->settings == nullptr) {
    impl_->settings = new_fluid_settings();
    if (impl_->settings != nullptr) {
      // Use the externally provided sample rate for offline
      // rendering so that FluidSynth matches the audio device.
      (void)fluid_settings_setnum(impl_->settings, "synth.sample-rate",
                                  impl_->sampleRate);
      // Explicitly disable FluidSynth's own audio driver by
      // selecting the "null" backend. JUCE is responsible for
      // audio I/O and pulls audio via render(), so we don't need
      // FluidSynth to initialise any system audio backend.
      (void)fluid_settings_setstr(impl_->settings, "audio.driver",
                                  "null");
    }
  }

  if (impl_->synth == nullptr && impl_->settings != nullptr) {
    impl_->synth = new_fluid_synth(impl_->settings);
  }
}

void SampleplaySynth::setSampleRate(const double sampleRate)
{
  if (sampleRate <= 0.0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_ == nullptr) {
    return;
  }

  if (std::abs(impl_->sampleRate - sampleRate) < 1e-3) {
    return;
  }

  impl_->sampleRate = sampleRate;

  // Recreate FluidSynth objects so that the new sample rate takes
  // effect. The caller is responsible for reloading any required
  // SoundFont afterwards.
  clearSynthLocked();
}

bool SampleplaySynth::loadSoundfont(const std::string& path,
                                    std::string* const error_message)
{
  if (error_message != nullptr) {
    *error_message = {};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_ == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Internal SampleplaySynth not initialised";
    }
    return false;
  }

  ensureSynthLocked();
  if (impl_->synth == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Failed to create FluidSynth synthesiser";
    }
    return false;
  }

  if (impl_->sfontId >= 0) {
    (void)fluid_synth_sfunload(impl_->synth, impl_->sfontId, 1);
    impl_->sfontId = -1;
  }

  const int sfid = fluid_synth_sfload(impl_->synth, path.c_str(), 1);
  if (sfid < 0) {
    if (error_message != nullptr) {
      *error_message = "FluidSynth could not load soundfont";
    }
    return false;
  }

  impl_->sfontId = sfid;
  impl_->currentPath = path;
  return true;
}

void SampleplaySynth::noteOn(const int bank, const int program,
                             const int midiKey,
                             const float velocity01)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_ == nullptr || impl_->synth == nullptr ||
      impl_->sfontId < 0) {
    return;
  }

  const int channel = 0;  // Single-channel use for now.

  const int clampedKey = std::clamp(midiKey, 0, 127);
  const float v = std::clamp(velocity01, 0.0F, 1.0F);
  int velocity = static_cast<int>(v * 127.0F);
  if (velocity <= 0) {
    velocity = 1;
  } else if (velocity > 127) {
    velocity = 127;
  }

  // Enforce simple monophonic behaviour on the single FluidSynth
  // channel used by Sampleplay: before triggering a new note, send
  // an "all notes off" so any previously playing key enters its
  // release phase. This prevents long or looping presets from
  // accumulating and sounding "infinite" when driven repeatedly by
  // the Sequencer.
  (void)fluid_synth_all_notes_off(impl_->synth, channel);

  // Select the requested preset on the fly before triggering the
  // note. This keeps the interface simple at the cost of a tiny
  // overhead per note-on, which is acceptable for the current
  // density of events.
  (void)fluid_synth_program_select(impl_->synth, channel,
                                   impl_->sfontId, bank, program);
  (void)fluid_synth_noteon(impl_->synth, channel, clampedKey,
                           velocity);
}

void SampleplaySynth::noteOff(const int midiKey)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_ == nullptr || impl_->synth == nullptr ||
      impl_->sfontId < 0) {
    return;
  }

  const int channel = 0;
  const int clampedKey = std::clamp(midiKey, 0, 127);
  (void)fluid_synth_noteoff(impl_->synth, channel, clampedKey);
}

void SampleplaySynth::render(float* const left, float* const right,
                             const int numSamples)
{
  if (left == nullptr || right == nullptr || numSamples <= 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (impl_ == nullptr || impl_->synth == nullptr ||
      impl_->sfontId < 0) {
    std::fill(left, left + numSamples, 0.0F);
    std::fill(right, right + numSamples, 0.0F);
    return;
  }

  (void)fluid_synth_write_float(impl_->synth, numSamples, left, 0, 1,
                                right, 0, 1);
}

}  // namespace rectai
