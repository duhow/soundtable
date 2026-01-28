#pragma once

#include <cstdint>

class OscSender;

namespace soundtable::tracker {

// Output mode used by soundtable-tracker to decide whether it should emit
// the current proprietary OSC format or TUIO 1.1 messages.
enum class TuioOutputMode {
    LegacyOsc,
    Tuio11
};

} // namespace soundtable::tracker
