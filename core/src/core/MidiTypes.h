#pragma once

#include <cstdint>

namespace soundtable {

// Lightweight internal representation of a MIDI note event.
// This is independent from JUCE but can be mapped to
// juce::MidiMessage or to engine-specific note triggers.
struct MidiNoteEvent {
  int channel{0};          // Logical MIDI channel (0-15 typical).
  int note{0};             // MIDI note number [0, 127].
  float velocity01{0.0F};  // Normalised velocity [0, 1].
  double timeBeats{0.0};   // Position in beats on the global timeline.
  bool is_note_on{true};   // true = NoteOn, false = NoteOff.
};

}  // namespace soundtable
