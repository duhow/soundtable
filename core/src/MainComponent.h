#pragma once

#include <string>
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
    [[nodiscard]] bool isInsideMusicArea(
        const rectai::ObjectInstance& obj) const;

    AudioEngine& audioEngine_;
    rectai::Scene scene_;

    // OSC bridge that updates the Scene from tracking messages.
    TrackingOscReceiver trackingOscReceiver_{scene_, 3333};

    // Tracking state for simple click-and-drag interaction.
    std::int64_t draggedObjectId_{0};

    // Simple per-object mute state (by tracking id).
    std::unordered_set<std::int64_t> mutedObjects_;

    // Per-connection mute state, keyed by a stable connection id.
    std::unordered_set<std::string> mutedConnections_;

    // Pairs of objects that are currently colliding (touching) and have
    // already triggered a hardlink toggle while in contact. Used to
    // detect new collision events as objects move.
    std::unordered_set<std::string> activeHardlinkCollisions_;

    // Module-level pairs for which an existing dynamic connection has been
    // temporarily promoted to a hardlink. When the hardlink is toggled
    // off again, these pairs restore their original non-hardlink
    // connection instead of removing it entirely.
    std::unordered_set<std::string> promotedHardlinkPairs_;

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

    // Per-instrument side controls (left: freq, right: gain).
    std::int64_t sideControlObjectId_{0};
    enum class SideControlKind { kNone = 0, kFreq = 1, kGain = 2 };
    SideControlKind sideControlKind_{SideControlKind::kNone};

    void toggleHardlinkBetweenObjects(std::int64_t objectIdA,
                                      std::int64_t objectIdB);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainComponent)
};
