#pragma once

#include <string>
#include <vector>

namespace soundtable {

// Lightweight description of a preset (instrument) inside a
// SoundFont2 (.sf2) file. The bank and program numbers follow the
// standard MIDI convention.
struct SoundfontPreset {
    int bank{0};
    int program{0};
    std::string name;
};

// Uses FluidSynth to open a SoundFont2 file and enumerate all
// presets (instruments) it contains. On success, fills
// `out_presets` with one entry per preset and returns true. On
// failure, returns false and, if `error_message` is not null,
// writes a short human-readable description.
bool EnumerateSoundfontPresets(const std::string& path,
                               std::vector<SoundfontPreset>& out_presets,
                               std::string* error_message = nullptr);

}  // namespace soundtable
