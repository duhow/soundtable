#pragma once

#include <juce_gui_extra/juce_gui_extra.h>

class MainComponent : public juce::Component {
public:
    MainComponent();
    ~MainComponent() override = default;

    void paint(juce::Graphics& g) override;
    void resized() override;

private:
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainComponent)
};
