#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <juce_gui_extra/juce_gui_extra.h>

#include "TrackingOscReceiver.h"
#include "core/AudioGraph.h"
#include "core/Scene.h"

class AudioEngine;

class MainComponent : public juce::Component, public juce::Timer {
public:
    explicit MainComponent(AudioEngine& audioEngine,
                           juce::String initialSessionPath = {});
    ~MainComponent() override = default;

    void paint(juce::Graphics& g) override;
    void resized() override;

    // Explicit mouseMove handler that does not trigger repaints by
    // itself. Plain hover events are handled via the base
    // juce::Component implementation; visual updates are driven by
    // the timer and explicit drag/click gestures.
    void mouseMove(const juce::MouseEvent& event) override;

    void mouseDown(const juce::MouseEvent& event) override;
    void mouseDrag(const juce::MouseEvent& event) override;
    void mouseUp(const juce::MouseEvent& event) override;
    void mouseWheelMove(const juce::MouseEvent& event,
                        const juce::MouseWheelDetails& wheel) override;

    void rotationTrackingUpdate(std::unordered_map<std::int64_t, float>* rotationDeltaDegrees);
    void timerCallback() override;

private:
    [[nodiscard]] bool isInsideMusicArea(
        const rectai::ObjectInstance& obj) const;

    // Recompute and cache the inside-music-area flag on all
    // objects in the Scene so that hot paths (paint, audio) can
    // query a simple attribute instead of re-evaluating the
    // geometry every time.
    void refreshInsideMusicAreaFlags();

    // Compute whether a given object lies inside the current
    // musical area based on the component bounds. This does not
    // mutate the Scene; it is used by refreshInsideMusicAreaFlags
    // and by input handlers when an object is being dragged.
    [[nodiscard]] bool computeInsideMusicArea(
        const rectai::ObjectInstance& obj) const;

    // Map an ObjectInstance in table space (centre-origin, radius 1)
    // to component-space pixels for a given bounds rectangle.
    static juce::Point<float> objectTableToScreen(
        const rectai::ObjectInstance& obj,
        const juce::Rectangle<float>& bounds);

    // Generic pointer handlers shared by mouse and external TUIO
    // cursor input.
    void handlePointerDown(juce::Point<float> position,
                           const juce::ModifierKeys& mods);
    void handlePointerDrag(juce::Point<float> position,
                           const juce::ModifierKeys& mods);
    void handlePointerUp(const juce::ModifierKeys& mods);

    // Entry points used by TrackingOscReceiver to mirror TUIO 2Dcur
    // events into the same interaction model as the mouse. The
    // coordinates are normalised in [0,1] over the full component
    // bounds and projected to pixels internally.
    void handleTuioCursorDown(float normX, float normY);
    void handleTuioCursorMove(float normX, float normY);
    void handleTuioCursorUp();

    void applyControlDropMuteIfNeeded(const juce::ModifierKeys& mods);

    bool loadAtlasResources();
    bool unloadAtlasResources();

    AudioEngine& audioEngine_;
    rectai::Scene scene_;

    // Logical audio graph derived from the Scene. This is the Phase 2
    // building block towards per-connection audio buffers: it captures
    // modules as nodes and connections as typed edges (audio/MIDI).
    rectai::AudioGraph audioGraph_;

    // OSC bridge that updates the Scene from tracking messages.
    TrackingOscReceiver trackingOscReceiver_{scene_, 3333};

    // Tracking state for simple click-and-drag interaction.
    std::int64_t draggedObjectId_{0};

    // Last known angle in degrees for each tracked object, used to
    // compute per-frame rotation deltas that modulate module
    // frequency parameters.
    std::unordered_map<std::int64_t, float> lastObjectAngleDegrees_;

    // Per-connection mute state, keyed by a stable connection id. All
    // mute semantics (including the implicit route to the master) are
    // expressed as connection mutes: center→module lines map to the
    // auto-generated module→Output(-1) connection and module→module
    // lines to their explicit `Connection` entries in the `Scene`.
    std::unordered_set<std::string> mutedConnections_;

    // Pairs of objects that are currently colliding (touching) and have
    // already triggered a hardlink toggle while in contact. Used to
    // detect new collision events as objects move.
    std::unordered_set<std::string> activeHardlinkCollisions_;

    // Flag used to avoid mutating the initial hardlink topology on
    // session load: the first time we evaluate collisions we only
    // populate activeHardlinkCollisions_ without toggling, so that
    // modules that start already near each other (or near the centre
    // master) do not immediately change their hardlink state.
    bool hardlinkCollisionsInitialised_{false};

    // Module-level pairs for which an existing dynamic connection has been
    // temporarily promoted to a hardlink. When the hardlink is toggled
    // off again, these pairs restore their original non-hardlink
    // connection instead of removing it entirely.
    std::unordered_set<std::string> promotedHardlinkPairs_;

    // Transport / tempo visualisation.
    float bpm_{120.0F};
    double beatPhase_{0.0};
    int beatIndex_{0};

    // Timestamp (seconds, hi-res clock) of the last time the global
    // BPM value changed in response to user interaction. Used by the
    // paint code to show a transient BPM label that fades out after a
    // short period of inactivity.
    double bpmLastChangeSeconds_{0.0};

    // Lightweight input activity indicator driven by OSC/TUIO
    // traffic. The UI shows a small label ("OSC" or "TUIO") near
    // the top-right corner of the component, just to the left of the
    // dock strip, along with a short-lived green dot that flashes on
    // each received message. The label disappears after 60 seconds
    // of inactivity.
    enum class InputActivityKind { kNone = 0, kOsc = 1, kTuio = 2 };
    InputActivityKind lastInputActivityKind_{InputActivityKind::kNone};
    double lastInputActivitySeconds_{0.0};
    double inputActivityPulseSeconds_{0.0};

    // Sampleplay instrument label visibility: per-module timestamp of
    // the last time the instrument was changed or the module entered
    // the musical area. Used by the paint code to fade out the
    // instrument title after a short period of inactivity.
    std::unordered_map<std::string, double>
        sampleplayLabelLastChangeSeconds_;

    // Loop sample label visibility: per-module timestamp of the last
    // time the selected slot changed or the module entered the
    // musical area. Used by the paint code to fade out the currently
    // selected loop filename after a short period of inactivity.
    std::unordered_map<std::string, double>
        loopLabelLastChangeSeconds_;

    // Last timer tick timestamp (seconds) used to derive dt for
    // animations, so visuals remain stable if the timer frequency
    // changes.
    double lastTimerSeconds_{0.0};

    // Last repaint timestamp (seconds) used to cap the maximum UI
    // refresh rate independently of the timer frequency. This allows
    // us to run audio/logic updates at a higher rate while keeping
    // the more expensive paint calls at a lower, but still smooth,
    // cadence.
    double lastRepaintSeconds_{0.0};

    // Cached background image for the static table geometry
    // (black backdrop, coloured disc and soft outer ring). This is
    // rendered only when the component bounds or the table colour
    // change, so that hot paint paths can simply blit the cached
    // image instead of rebuilding gradients and edge tables every
    // frame.
    juce::Image tableBackgroundCache_;
    bool tableBackgroundDirty_{true};

    // Cached background panel for the right-hand dock strip
    // (rounded rectangle + outline + static "Dock" title). The
    // dynamic dock contents (bubbles, icons, BPM labels) are still
    // drawn every frame, but the expensive panel geometry and text
    // are rendered once per size change.
    juce::Image dockBackgroundCache_;
    bool dockBackgroundDirty_{true};

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

    // Visual sequencer phase used only for UI widgets
    // (independent from the audio runtime).
    double sequencerPhase_{0.0};
    int sequencerStep_{0};

    // Audio sequencer phase/step (16 steps per bar) used by the
    // monophonic runtime of the Sequencer module.
    double sequencerAudioPhase_{0.0};
    int sequencerAudioStep_{0};

    // Contador absoluto de steps de audio avanzados desde el inicio
    // del transporte. Se usa para garantizar que, si el timer se
    // retrasa y se saltan varios steps ideales entre dos llamadas a
    // `timerCallback`, se ejecuten todos los pasos intermedios en
    // orden en lugar de sólo el último, reduciendo saltos
    // perceptibles en el patrón rítmico.
    std::int64_t sequencerAudioStepCounter_{0};

    // Simple beat counter advancing with the global transport.
    // Used to tag MidiNoteEvent events with a beat position without
    // introducing a full MIDI scheduler yet.
    double transportBeats_{0.0};

    // When true, Sequencer steps modulate the volume (velocity) of
    // destination modules: note velocity in Sampleplay and the
    // per-module sequencer gain factor in Oscillator. When false,
    // the Sequencer only controls pitch/trigger and volume is
    // assumed to be controlled by other MIDI sources.
    bool sequencerControlsVolume_{true};

    // Per-oscillator gain factor driven by the Sequencer when
    // `sequencerControlsVolume_` is true. This represents the
    // instantaneous MIDI velocity mapped to [0,1] for each
    // Oscillator module and is applied on top of the user-facing
    // `gain` parameter (white handle). The effective audio gain is
    // the minimum of the module `gain` and this sequencer factor,
    // and the corresponding grey handle in the UI cannot exceed the
    // white handle.
    std::unordered_map<std::string, float> oscillatorSequencerGain_;

    // Set of module ids that are currently contributing audible audio
    // to the master bus. Used by the visual layer to decide which
    // lines should display a waveform instead of a plain line.
    std::unordered_set<std::string> modulesWithActiveAudio_;

    // Mapping from module id to the AudioEngine voice index currently
    // representing its audio chain (generator and optional downstream
    // module). This lets the paint code fetch a module-specific
    // waveform instead of relying on the global mix only.
    std::unordered_map<std::string, int> moduleVoiceIndex_;

    // Visual source mapping per audio connection. This is the first
    // step towards conceptual "per-connection buffers": rather than
    // deciding ad-hoc in the paint code whether a connection should
    // use pre- or post-filter signal (or Sampleplay), the audio-side
    // logic fills this map based on the Scene and the module→voice
    // mapping. Later, when an explicit audio graph with per-connection
    // taps exists, this structure will be able to point at real
    // per-connection buffers in the engine.
    struct ConnectionVisualSource {
        enum class Kind { kNone = 0, kVoicePre, kVoicePost, kSampleplay };
        Kind kind{Kind::kNone};
        int voiceIndex{-1};  // válido para kVoicePre/kVoicePost
    };

    std::unordered_map<std::string, ConnectionVisualSource>
        connectionVisualSources_;

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

    // Cache of pre-scaled atlas icons keyed by sprite id and
    // destination size. This avoids re-sampling the large atlas
    // image on every frame when drawing module and dock icons; the
    // cached images remain in SingleChannel format so they can still
    // be tinted at paint time via Graphics::setColour.
    std::unordered_map<std::string, juce::Image> atlasIconCache_;

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

    void invalidateTableBackground();
    void renderTableBackgroundIfNeeded(const juce::Rectangle<int>& bounds);

    // Request a repaint while enforcing a global rate-limit so
    // that the UI does not exceed the target frame rate even when
    // multiple sources (timer, input handlers) trigger repaints.
    void repaintWithRateLimit();

    void invalidateDockBackground();
    void renderDockBackgroundIfNeeded(
        const juce::Rectangle<int>& dockBounds);

    // Retrieve or lazily create a pre-scaled SingleChannel icon
    // image for the given atlas sprite id and destination size. The
    // returned image can be drawn with drawImageAt after setting the
    // desired tint colour on the Graphics context. Returns an
    // invalid image if the sprite is not found or the atlas is not
    // loaded.
    [[nodiscard]] juce::Image getCachedAtlasIcon(const std::string& iconId,
                                                 int destWidth,
                                                 int destHeight);

    void toggleHardlinkBetweenObjects(std::int64_t objectIdA,
                                      std::int64_t objectIdB);

    // Trigger Sampleplay notes (via FluidSynth in AudioEngine) on
    // every beat of the global tempo. `strongBeat` is true on the
    // downbeat of each 4-beat bar and can be used to slightly accent
    // the velocity.
    void triggerSampleplayNotesOnBeat(bool strongBeat);

    // Mark the Sampleplay instrument label for a given module id as
    // recently active so that the UI keeps it visible and restarts
    // its fade-out timer.
    void markSampleplayInstrumentLabelActive(const std::string& moduleId);

    // Mark the Loop sample label for a given module id as recently
    // active so that the UI keeps it visible and restarts its
    // fade-out timer.
    void markLoopSampleLabelActive(const std::string& moduleId);

    // Touch interface state.
    bool isTouchActive_{false};
    bool isTouchHeld_{false};
    bool touchStartedInDock_{false};

    // Explicit interaction mode flag: when true, the current gesture
    // is interpreted as a "cut" gesture (red cursor + trail) that
    // can mute lines by crossing them. When false, the gesture is a
    // regular interaction (drag modules, adjust side controls, click
    // and hold on lines, etc.) rendered with a white cursor.
    bool isCutModeActive_{false};
    juce::Point<float> currentTouchPosition_;

    struct TrailPoint {
        juce::Point<float> position;
        double timestamp;  // High-resolution timestamp in seconds.
    };
    std::vector<TrailPoint> touchTrail_;
    static constexpr int kMaxTrailPoints = 500;
    static constexpr bool kEnableTrailFade = true;
    static constexpr double kTrailFadeDurationSeconds = 0.4;

    // Line cutting with touch: track connections and object-to-center
    // lines that have been "cut" during the current touch drag and
    // should toggle their mute state when the touch is released.
    std::unordered_set<std::string> touchCutConnections_;
    std::unordered_set<std::int64_t> touchCutObjects_;
    
    // Track which lines are currently being intersected (in the
    // intersection zone) to detect when we enter/exit and toggle only
    // on transitions.
    std::unordered_set<std::string> touchCurrentlyIntersectingConnections_;
    std::unordered_set<std::int64_t> touchCurrentlyIntersectingObjects_;

    // Click-and-hold mute state: when the user clicks on a sound line,
    // the module is muted while the button is held down. The split point
    // (normalized 0-1) along the connection determines where the waveform
    // visibility transitions to dashed line.
    struct ConnectionHoldState {
        std::string connection_key;
        std::int64_t object_id{0};   // For object-to-center lines.
        bool is_object_line{false};  // True if object-to-center, false if module-to-module.
        float split_point{0.0F};     // Normalized position along the line (0=source, 1=destination).
        // When true, releasing the hold should unmute the underlying
        // connection or radial if it was muted at the start of the
        // gesture. This lets users click/hold on an already-muted
        // line to clear its mute without performing a cut gesture.
        bool unmute_on_release{false};
    };
    std::optional<ConnectionHoldState> activeConnectionHold_;

    // Rebuild the mapping from Scene::connections() to visual
    // waveform sources (pre/post voice or Sampleplay) using the
    // current module→voice assignment and module metadata. This lets
    // the paint code query a single source per connection instead of
    // reimplementing routing heuristics.
    void updateConnectionVisualSources();

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainComponent)
};
