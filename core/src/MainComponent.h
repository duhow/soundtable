#pragma once

#include <unordered_set>
#include <vector>

#include <juce_gui_extra/juce_gui_extra.h>

#include "TrackingOscReceiver.h"
#include "core/Scene.h"

class AudioEngine;

class MainComponent : public juce::Component, public juce::Timer {
public:
    explicit MainComponent(AudioEngine& audioEngine);
    ~MainComponent() override = default;

    void paint(juce::Graphics& g) override;
    void resized() override;

    void mouseDown(const juce::MouseEvent& event) override;
    void mouseDrag(const juce::MouseEvent& event) override;
    void mouseUp(const juce::MouseEvent& event) override;

    void timerCallback() override;

private:
    AudioEngine& audioEngine_;
    rectai::Scene scene_;

    // OSC bridge that updates the Scene from tracking messages.
    TrackingOscReceiver trackingOscReceiver_{scene_, 3333};

    // Tracking state for simple click-and-drag interaction.
    std::int64_t draggedObjectId_{0};

    // Simple per-object mute state (by tracking id).
    std::unordered_set<std::int64_t> mutedObjects_;

    // Transport / tempo visualisation.
    double bpm_{120.0};
    double beatPhase_{0.0};
    int beatIndex_{0};

    struct Pulse {
        float age{0.0F};
        bool strong{false};
    };

    std::vector<Pulse> pulses_;

    // Animation phases.
    double connectionFlowPhase_{0.0};
    double sequencerPhase_{0.0};
    int sequencerStep_{0};

    // Selection state for radial menu.
    std::int64_t selectedObjectId_{0};

    // Radial slider interaction state (index: 0=freq, 1=gain, 2=fx).
    bool draggingRadialSlider_{false};
    int draggingSliderIndex_{-1};

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainComponent)
};
