#pragma once

#include <functional>
#include <cmath>

#include <juce_graphics/juce_graphics.h>

#include "core/AudioModules.h"

// Canonical ADSR envelope time limits in milliseconds. These mirror
// the clamping used when loading Reactable patches so that the UI
// stays in sync with the underlying model and loader logic.
inline constexpr float kModuleEnvelopeMaxAttackMs = 2000.0F;
inline constexpr float kModuleEnvelopeMaxDecayMs = 2000.0F;
inline constexpr float kModuleEnvelopeMaxDurationMs = 7500.0F;
inline constexpr float kModuleEnvelopeMaxReleaseMs = 2000.0F;

// Helpers to render and interact with the per-module Envelope (ADSR)
// view inside the module detail panel.

struct ModuleEnvelopeGeometry {
    juce::Rectangle<float> contentBounds;
    juce::Rectangle<float> barsArea;
    juce::Rectangle<float> bottomBar;
};

// Map a linear normalised envelope value in [0,1] to a visual
// height in [0,1] using a non-linear curve. The current mapping is
// sqrt(v) so that, por ejemplo, 125/2000 ≈ 0.0625 se renderiza con
// ~25% de altura y 20/2000 ≈ 0.01 con ~10%.
inline float moduleEnvelopeVisualFromNormalised(float v) noexcept
{
    if (!std::isfinite(v) || v <= 0.0F) {
        return 0.0F;
    }
    if (v >= 1.0F) {
        return 1.0F;
    }
    return std::sqrt(v);
}

// Inverse mapping: given a visual height fraction in [0,1], recover
// the underlying linear normalised value so that pointer drags map
// consistently to envelope times.
inline float moduleEnvelopeNormalisedFromVisual(float visual01) noexcept
{
    if (!std::isfinite(visual01) || visual01 <= 0.0F) {
        return 0.0F;
    }
    if (visual01 >= 1.0F) {
        return 1.0F;
    }
    return visual01 * visual01;
}

// Compute the layout for the envelope view inside a panel rectangle.
ModuleEnvelopeGeometry computeModuleEnvelopeGeometry(
    const juce::Rectangle<float>& panelBounds);

// Kind of interaction hit within the envelope view.
enum class ModuleEnvelopeHitKind {
    kNone = 0,
    kPresetLeft,
    kPresetRight,
    kBar,
};

// Result of hit-testing a pointer position against the envelope view.
// For bar hits, `barIndex` is 0..3 (A,D,S,R) and `value01` is the
// normalised height in [0,1] derived from the pointer Y position
// (0 at the bottom, 1 at the top of the bar area).
struct ModuleEnvelopeHitResult {
    ModuleEnvelopeHitKind kind{ModuleEnvelopeHitKind::kNone};
    int barIndex{-1};
    float value01{0.0F};
};

// Paint the ADSR envelope view (4 vertical bars + bottom preset
// buttons) inside the given panel rectangle. Time values are
// normalised against the provided maxima in milliseconds.
void paintModuleEnvelopeView(
    juce::Graphics& g,
    const juce::Rectangle<float>& panelBounds,
    const soundtable::AudioModuleWithEnvelope& module,
    bool atlasLoaded,
    const std::function<juce::Image(const std::string&, int, int)>&
        getCachedIcon,
    float attackMaxMs,
    float decayMaxMs,
    float durationMaxMs,
    float releaseMaxMs);

// Hit-test a pointer position in panel-local coordinates against the
// envelope view. Returns the kind of element under the pointer and, for
// bars, the corresponding index and normalised value.
ModuleEnvelopeHitResult hitTestModuleEnvelope(
    const juce::Rectangle<float>& panelBounds,
    juce::Point<float> localPos,
    float attackMaxMs,
    float decayMaxMs,
    float durationMaxMs,
    float releaseMaxMs);
