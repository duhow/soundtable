#include "MainComponent_XYControl.h"

namespace soundtable::ui {

namespace {
inline float clamp01(float v) noexcept
{
    return juce::jlimit(0.0F, 1.0F, v);
}
}  // namespace

XYControl::XYControl()
{
    setInterceptsMouseClicks(true, false);
}

void XYControl::setOnValueChanged(
    std::function<void(float, float)> callback)
{
    onValueChanged_ = std::move(callback);
}

void XYControl::setVisualMinimums(const float minX, const float minY)
{
    const float clampedMinX = clamp01(minX);
    const float clampedMinY = clamp01(minY);

    if (std::abs(minVisualX_ - clampedMinX) < 1.0e-3F &&
        std::abs(minVisualY_ - clampedMinY) < 1.0e-3F) {
        return;
    }

    minVisualX_ = clampedMinX;
    minVisualY_ = clampedMinY;

    // Ensure the current visual position respects the new minimums.
    setVisualPosition(visualX_, visualY_, false);
}

void XYControl::setNormalisedPosition(const float x01, const float y01,
                                      const bool notify)
{
    const float clampedX = clamp01(x01);
    const float clampedY = clamp01(y01);

    const float spanX = std::max(0.0001F, 1.0F - minVisualX_);
    const float spanY = std::max(0.0001F, 1.0F - minVisualY_);

    const float visualX = minVisualX_ + clampedX * spanX;
    // Invert logical Y so that higher values move the handle upwards.
    const float visualY = 1.0F - spanY * clampedY;

    setVisualPosition(visualX, visualY, notify);
}

float XYControl::normalisedX() const noexcept
{
    const float spanX = std::max(0.0001F, 1.0F - minVisualX_);
    return clamp01((visualX_ - minVisualX_) / spanX);
}

float XYControl::normalisedY() const noexcept
{
    const float spanY = std::max(0.0001F, 1.0F - minVisualY_);
    // Map from visual [minVisualY_,1] (top->bottom) to logical
    // [1,0] so that dragging upwards increases the value.
    return clamp01((1.0F - visualY_) / spanY);
}

void XYControl::setVisualPosition(const float visualX,
                                  const float visualY,
                                  const bool fromUser)
{
    const float clampedX = juce::jlimit(minVisualX_, 1.0F, visualX);
    const float clampedY = juce::jlimit(minVisualY_, 1.0F, visualY);

    if (std::abs(visualX_ - clampedX) < 1.0e-3F &&
        std::abs(visualY_ - clampedY) < 1.0e-3F) {
        return;
    }

    visualX_ = clampedX;
    visualY_ = clampedY;

    if (fromUser && onValueChanged_) {
        onValueChanged_(normalisedX(), normalisedY());
    }

    repaint();
}

void XYControl::updateFromLocalPosition(const float x, const float y,
                                        const bool isGestureEnd)
{
    const auto bounds = getLocalBounds().toFloat();
    if (bounds.getWidth() <= 0.0F || bounds.getHeight() <= 0.0F) {
        return;
    }

    const float localX = juce::jlimit(bounds.getX(), bounds.getRight(), x);
    const float localY = juce::jlimit(bounds.getY(), bounds.getBottom(), y);

    const float normX = (localX - bounds.getX()) / bounds.getWidth();
    const float normY = (localY - bounds.getY()) / bounds.getHeight();

    juce::ignoreUnused(isGestureEnd);
    setVisualPosition(clamp01(normX), clamp01(normY), true);
}

void XYControl::beginPointerAt(const float x, const float y)
{
    isDragging_ = true;
    updateFromLocalPosition(x, y, false);
}

void XYControl::dragPointerTo(const float x, const float y)
{
    if (!isDragging_) {
        return;
    }
    updateFromLocalPosition(x, y, false);
}

void XYControl::endPointerAt(const float x, const float y)
{
    if (!isDragging_) {
        return;
    }
    isDragging_ = false;
    updateFromLocalPosition(x, y, true);
}

void XYControl::paint(juce::Graphics& g)
{
    const auto bounds = getLocalBounds().toFloat();
    // Grid lines for orientation (no extra padded background).
    g.setColour(juce::Colours::white.withAlpha(0.06F));
    const float midX = bounds.getX() + bounds.getWidth() * 0.5F;
    const float midY = bounds.getY() + bounds.getHeight() * 0.5F;
    g.drawLine(midX, bounds.getY(), midX, bounds.getBottom(), 1.0F);
    g.drawLine(bounds.getX(), midY, bounds.getRight(), midY, 1.0F);

    // Handle position in pixels.
    const float handleRadius =
        0.08F * std::min(bounds.getWidth(), bounds.getHeight());
    const float clampedRadius = juce::jlimit(3.0F, 5.0F, handleRadius);

    const float px = bounds.getX() + bounds.getWidth() * visualX_;
    const float py = bounds.getY() + bounds.getHeight() * visualY_;

    juce::Rectangle<float> handleBounds(
        px - clampedRadius, py - clampedRadius,
        clampedRadius * 2.0F, clampedRadius * 2.0F);

    g.setColour(juce::Colours::white.withAlpha(0.95F));
    g.fillEllipse(handleBounds);
}

void XYControl::mouseDown(const juce::MouseEvent& event)
{
    beginPointerAt(event.position.x, event.position.y);
}

void XYControl::mouseDrag(const juce::MouseEvent& event)
{
    dragPointerTo(event.position.x, event.position.y);
}

void XYControl::mouseUp(const juce::MouseEvent& event)
{
    juce::ignoreUnused(event);
    if (isDragging_) {
        isDragging_ = false;
    }
}

}  // namespace soundtable::ui
