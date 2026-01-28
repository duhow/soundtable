#pragma once

#include <cstdint>
#include <string>

#include <juce_graphics/juce_graphics.h>

#include "core/Scene.h"

namespace soundtable::ui {

// Convert a 32-bit ARGB integer into a JUCE colour.
juce::Colour colourFromArgb(std::uint32_t argb);

// Build a stable textual key that identifies a connection uniquely.
std::string makeConnectionKey(const soundtable::Connection& connection);

// Build a stable key for an unordered pair of object ids
// (order-independent).
std::string makeObjectPairKey(std::int64_t a, std::int64_t b);

// Build a stable key for a directed pair of module ids (from>to).
std::string makeModulePairKey(const std::string& fromId,
                              const std::string& toId);
std::string makeModulePairKey(const Connection& conn);

// Returns true if `toObj` lies inside a ~120º cone whose vertex is at
// the centre of the table and whose axis points towards `fromObj`.
bool isConnectionGeometricallyActive(const soundtable::ObjectInstance& fromObj,
                                     const soundtable::ObjectInstance& toObj);

// Check if line segment (p1->p2) intersects with segment (q1->q2).
// Uses a threshold distance to account for near-misses (touching).
bool lineSegmentsIntersect(const juce::Point<float>& p1,
                          const juce::Point<float>& p2,
                          const juce::Point<float>& q1,
                          const juce::Point<float>& q2,
                          float threshold = 5.0F);

// Try to locate a Reactable resource file given a relative path such as
// "Resources/default.rtp". The helper will internally prefix the path
// with "com.reactable/" so that callers only need to specify the
// subfolder and file within the Reactable content tree. The search is performed
// relative to the current working directory and to the executable
// location, attempting a series of plausible prefixes.
//
// Returns a valid juce::File if found; otherwise returns an empty file
// (for which existsAsFile() will be false).
[[nodiscard]] juce::File loadFile(const juce::String& relativePath);

bool generateConnectionFromModules(
    const AudioModule& moduleA,
    const AudioModule& moduleB,
    bool isHardlink,
    Connection& outConnection);

}  // namespace soundtable::ui
