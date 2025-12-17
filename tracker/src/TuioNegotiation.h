#pragma once

#include <cstdint>

class OscSender;

namespace rectai::tracker {

// Output mode used by rectai-tracker to decide whether it should emit
// the current proprietary OSC format or TUIO 1.1 messages.
enum class TuioOutputMode {
    LegacyOsc,
    Tuio11
};

} // namespace rectai::tracker
