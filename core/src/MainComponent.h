#pragma once

#include <juce_gui_extra/juce_gui_extra.h>

#include "TrackingOscReceiver.h"
#include "core/Scene.h"

class MainComponent : public juce::Component {
public:
    MainComponent();
    ~MainComponent() override = default;

    void paint(juce::Graphics& g) override;
    void resized() override;

    void mouseDown(const juce::MouseEvent& event) override;
    void mouseDrag(const juce::MouseEvent& event) override;
    void mouseUp(const juce::MouseEvent& event) override;

private:
    rectai::Scene scene_;

    // OSC bridge that updates the Scene from tracking messages.
    TrackingOscReceiver trackingOscReceiver_{scene_, 3333};

    // Tracking state for simple click-and-drag interaction.
    std::int64_t draggedObjectId_{0};

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainComponent)
};
