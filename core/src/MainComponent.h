#pragma once

#include <string>
#include <unordered_map>
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

    bool loadAtlasResources();

    AudioEngine& audioEngine_;
    rectai::Scene scene_;

    // OSC bridge that updates the Scene from tracking messages.
    TrackingOscReceiver trackingOscReceiver_{scene_, 3333};

    // Tracking state for simple click-and-drag interaction.
    std::int64_t draggedObjectId_{0};

    // Simple per-object mute state (by tracking id).
    std::unordered_set<std::int64_t> mutedObjects_;

    // Last known angle in degrees for each tracked object, used to
    // compute per-frame rotation deltas that modulate module
    // frequency parameters.
    std::unordered_map<std::int64_t, float> lastObjectAngleDegrees_;

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

    // Timestamp (seconds, hi-res clock) of the last time the global
    // BPM value changed in response to user interaction. Used by the
    // paint code to show a transient BPM label that fades out after a
    // short period of inactivity.
    double bpmLastChangeSeconds_{0.0};

    // Last timer tick timestamp (seconds) used to derive dt for
    // animations, so visuals remain stable if the timer frequency
    // changes.
    double lastTimerSeconds_{0.0};

    // Master output (type=Output in .rtp) presentation state.
    juce::Colour masterColour_{juce::Colours::white};
    bool masterMuted_{false};

    struct Pulse {
        float age{0.0F};
        bool strong{false};
    };

    std::vector<Pulse> pulses_;

    // Animation phases.
    double connectionFlowPhase_{0.0};
    double sequencerPhase_{0.0};
    int sequencerStep_{0};

    // Set of module ids that are currently contributing audible audio
    // to the master bus. Used by the visual layer to decide which
    // lines should display a waveform instead of a plain line.
    std::unordered_set<std::string> modulesWithActiveAudio_;

    // Mapping from module id to the AudioEngine voice index currently
    // representing its audio chain (generator and optional downstream
    // module). This lets the paint code fetch a module-specific
    // waveform instead of relying on the global mix only.
    std::unordered_map<std::string, int> moduleVoiceIndex_;

    // Per-instrument side controls (left: freq, right: gain).
    std::int64_t sideControlObjectId_{0};
    enum class SideControlKind { kNone = 0, kFreq = 1, kGain = 2 };
    SideControlKind sideControlKind_{SideControlKind::kNone};

    struct AtlasSprite {
        juce::Rectangle<int> bounds;
    };

    juce::Image atlasImage_;
    std::unordered_map<std::string, AtlasSprite> atlasSprites_;
    bool atlasLoaded_{false};

    // Dock (right-hand strip) scroll state.
    float dockScrollOffset_{0.0F};
    bool isDraggingDockScroll_{false};
    float dockLastDragY_{0.0F};

    // Dock width calculation constants.
    static constexpr float kDockMaxWidth = 100.0F;
    static constexpr float kDockWidthRatio = 0.20F;

    [[nodiscard]] float calculateDockWidth(float boundsWidth) const {
        return juce::jmin(kDockMaxWidth, boundsWidth * kDockWidthRatio);
    }

    void toggleHardlinkBetweenObjects(std::int64_t objectIdA,
                                      std::int64_t objectIdB);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainComponent)
};
