#pragma once

#include <functional>

#include <juce_gui_basics/juce_gui_basics.h>

namespace soundtable::ui {

// Generic 2D XY pad control used inside module detail panels.
// The control exposes a normalised logical domain [0,1]x[0,1] while
// allowing callers to define per-axis visual minimums so that the
// on-screen handle never reaches the absolute left/top edge. Values
// reported via callbacks are always in logical [0,1] space.
class XYControl : public juce::Component {
public:
    XYControl();
    ~XYControl() override = default;

    void paint(juce::Graphics& g) override;
    void mouseDown(const juce::MouseEvent& event) override;
    void mouseDrag(const juce::MouseEvent& event) override;
    void mouseUp(const juce::MouseEvent& event) override;

    // Logical value in [0,1]x[0,1], independent from visual minimums.
    void setNormalisedPosition(float x01, float y01, bool notify = false);
    [[nodiscard]] float normalisedX() const noexcept;
    [[nodiscard]] float normalisedY() const noexcept;

    // Visual minimum per axis in [0,1). The handle will never move
    // to a visual position smaller than these thresholds, but the
    // reported logical value still maps the full [0,1] range.
    void setVisualMinimums(float minX, float minY);
    [[nodiscard]] float visualMinX() const noexcept { return minVisualX_; }
    [[nodiscard]] float visualMinY() const noexcept { return minVisualY_; }

    void setOnValueChanged(std::function<void(float /*x01*/, float /*y01*/)> callback);

    // Pointer helpers mirroring mouseDown/Drag/Up when the control is
    // embedded into a larger custom-painted surface instead of being
    // added as a child Component. Coordinates are expressed in the
    // control's local space after any external translation.
    void beginPointerAt(float x, float y);
    void dragPointerTo(float x, float y);
    void endPointerAt(float x, float y);

private:
    void updateFromLocalPosition(float x, float y, bool isGestureEnd);
    void setVisualPosition(float visualX, float visualY, bool fromUser);

    // Visual position in [0,1]x[0,1], clamped to
    // [minVisualX_,1]x[minVisualY_,1].
    float visualX_{0.5F};
    float visualY_{0.5F};

    // Per-axis visual minimums in [0,1). These control how far
    // towards the left/top the handle can move visually; logical
    // values always span [0,1] regardless of these thresholds.
    float minVisualX_{0.0F};
    float minVisualY_{0.0F};

    bool isDragging_{false};

    std::function<void(float, float)> onValueChanged_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(XYControl)
};

}  // namespace soundtable::ui
