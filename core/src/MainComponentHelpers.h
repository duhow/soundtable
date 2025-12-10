#pragma once

#include <cstdint>
#include <string>

#include <juce_graphics/juce_graphics.h>

#include "core/Scene.h"

namespace rectai::ui {

// Convert a 32-bit ARGB integer into a JUCE colour.
juce::Colour colourFromArgb(std::uint32_t argb);

// Build a stable textual key that identifies a connection uniquely.
std::string makeConnectionKey(const rectai::Connection& connection);

// Build a stable key for an unordered pair of object ids
// (order-independent).
std::string makeObjectPairKey(std::int64_t a, std::int64_t b);

// Build a stable key for a directed pair of module ids (from>to).
std::string makeModulePairKey(const std::string& fromId,
                              const std::string& toId);

// Returns true if `toObj` lies inside a 120º cone whose vertex is at
// the centre of the table and whose axis points towards `fromObj`.
bool isConnectionGeometricallyActive(const rectai::ObjectInstance& fromObj,
                                     const rectai::ObjectInstance& toObj);

}  // namespace rectai::ui
