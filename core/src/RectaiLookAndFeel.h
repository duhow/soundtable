#pragma once

#include <juce_gui_extra/juce_gui_extra.h>

// Simple LookAndFeel that, when a bundled TTF is present, forces
// JUCE to use that typeface as the default sans-serif font. This
// avoids querying all system fonts via fontconfig on startup.
//
// The font file is not part of the repository on purpose (see
// DEPENDENCIES.md / licensing). At runtime we look for a
// "rectai-default.ttf" file next to the resources so that the
// application only needs to open that single file instead of
// scanning /usr/share/fonts.

class RectaiLookAndFeel : public juce::LookAndFeel_V4 {
public:
    RectaiLookAndFeel();

private:
    juce::Typeface::Ptr embeddedTypeface_;
};
