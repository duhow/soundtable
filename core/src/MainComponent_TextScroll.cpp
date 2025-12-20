#include "MainComponent_TextScroll.h"

namespace rectai::ui {

TextScrollList::TextScrollList()
{
    setInterceptsMouseClicks(true, false);
}

void TextScrollList::setItems(const std::vector<Item>& items)
{
    items_ = items;
    if (selectedIndex_ >= static_cast<int>(items_.size())) {
        selectedIndex_ = -1;
    }
    updateScrollRange();
    repaint();
}

void TextScrollList::setItems(std::vector<Item>&& items)
{
    items_ = std::move(items);
    if (selectedIndex_ >= static_cast<int>(items_.size())) {
        selectedIndex_ = -1;
    }
    updateScrollRange();
    repaint();
}

void TextScrollList::clearItems()
{
    items_.clear();
    selectedIndex_ = -1;
    updateScrollRange();
    repaint();
}

int TextScrollList::getNumItems() const noexcept
{
    return static_cast<int>(items_.size());
}

const TextScrollList::Item* TextScrollList::getItem(int index) const noexcept
{
    if (index < 0 || index >= static_cast<int>(items_.size())) {
        return nullptr;
    }
    return &items_[static_cast<std::size_t>(index)];
}

void TextScrollList::setSelectedIndex(int index, const bool notify)
{
    if (index < 0 || index >= static_cast<int>(items_.size())) {
        index = -1;
    }

    if (selectedIndex_ == index) {
        return;
    }

    selectedIndex_ = index;
    repaint();

    if (notify && onSelectionChanged_) {
        onSelectionChanged_(selectedIndex_);
    }
}

int TextScrollList::getSelectedIndex() const noexcept
{
    return selectedIndex_;
}

void TextScrollList::setOnSelectionChanged(
    std::function<void(int)> callback)
{
    onSelectionChanged_ = std::move(callback);
}

void TextScrollList::setMaxVisibleItems(const int maxItems)
{
    const int clamped = maxItems <= 0 ? 1 : maxItems;
    if (maxVisibleItems_ == clamped) {
        return;
    }
    maxVisibleItems_ = clamped;
    updateScrollRange();
    repaint();
}

int TextScrollList::getMaxVisibleItems() const noexcept
{
    return maxVisibleItems_;
}

void TextScrollList::setRowHeight(const float height)
{
    const float clamped = height < 4.0F ? 4.0F : height;
    if (std::abs(rowHeight_ - clamped) < 0.01F) {
        return;
    }
    rowHeight_ = clamped;
    updateScrollRange();
    repaint();
}

float TextScrollList::getRowHeight() const noexcept
{
    return rowHeight_;
}

void TextScrollList::scrollToTop()
{
    setScrollOffset(0.0F);
}

void TextScrollList::scrollToBottom()
{
    setScrollOffset(maxScrollOffset_);
}

void TextScrollList::updateScrollRange()
{
    const float totalHeight =
        static_cast<float>(items_.size()) * rowHeight_;
    const float visibleHeight =
        static_cast<float>(maxVisibleItems_) * rowHeight_;

    maxScrollOffset_ = juce::jmax(0.0F, totalHeight - visibleHeight);
    scrollOffset_ = juce::jlimit(0.0F, maxScrollOffset_, scrollOffset_);
}

void TextScrollList::setScrollOffset(const float newOffset)
{
    const float clamped = juce::jlimit(0.0F, maxScrollOffset_, newOffset);
    if (std::abs(scrollOffset_ - clamped) < 0.01F) {
        return;
    }
    scrollOffset_ = clamped;
    repaint();
}

void TextScrollList::paint(juce::Graphics& g)
{
    const auto bounds = getLocalBounds().toFloat();

    g.setColour(juce::Colours::white.withAlpha(0.03F));
    g.fillRoundedRectangle(bounds, 4.0F);

    const float maxVisibleHeight =
        rowHeight_ * static_cast<float>(maxVisibleItems_);
    const float viewHeight =
        juce::jmin(bounds.getHeight(), maxVisibleHeight);

    const int firstIndex = static_cast<int>(scrollOffset_ / rowHeight_);
    const float firstOffset =
        std::fmod(scrollOffset_, rowHeight_);  // in [0,rowHeight_)

    float y = -firstOffset;
    const int numItems = getNumItems();

    for (int i = firstIndex; i < numItems; ++i) {
        if (y >= viewHeight) {
            break;
        }

        juce::Rectangle<float> rowBounds(
            bounds.getX(), bounds.getY() + y, bounds.getWidth(),
            rowHeight_);

        if (rowBounds.getBottom() < bounds.getY()) {
            y += rowHeight_;
            continue;
        }

        if (i == selectedIndex_) {
            g.setColour(juce::Colours::white.withAlpha(0.18F));
            g.fillRect(rowBounds);
        }

        const auto& item = items_[static_cast<std::size_t>(i)];

        juce::Font font(13.0F);
        if (item.style.bold) {
            font.setBold(true);
        }
        g.setFont(font);

        juce::Colour textColour =
            item.style.useCustomColour
                ? item.style.colour
                : juce::Colours::white.withAlpha(0.85F);
        g.setColour(textColour);

        auto textBounds = rowBounds.toNearestInt();
        textBounds.reduce(6, 0);

        g.drawFittedText(item.text, textBounds,
                         juce::Justification::centredLeft, 1);

        y += rowHeight_;
    }
}

void TextScrollList::mouseDown(const juce::MouseEvent& event)
{
    handleClickAt(event.position.x, event.position.y);
}

void TextScrollList::mouseWheelMove(
    const juce::MouseEvent& event,
    const juce::MouseWheelDetails& wheel)
{
    juce::ignoreUnused(event);
    handleWheelDelta(wheel.deltaY);
}

void TextScrollList::handleClickAt(const float x, const float y)
{
    juce::ignoreUnused(x);

    const float maxVisibleHeight =
        rowHeight_ * static_cast<float>(maxVisibleItems_);
    const float viewHeight =
        juce::jmin(static_cast<float>(getHeight()), maxVisibleHeight);

    const float localY = y;
    if (localY < 0.0F || localY >= viewHeight) {
        return;
    }

    const float absoluteY = localY + scrollOffset_;
    const int index = static_cast<int>(absoluteY / rowHeight_);

    if (index < 0 || index >= getNumItems()) {
        return;
    }

    setSelectedIndex(index, true);
}

void TextScrollList::handleWheelDelta(const float deltaY)
{
    if (maxScrollOffset_ <= 0.0F) {
        return;
    }

    const float step = rowHeight_ * 3.0F;
    const float delta = -deltaY * step;

    if (std::abs(delta) < 0.001F) {
        return;
    }

    setScrollOffset(scrollOffset_ + delta);
}

}  // namespace rectai::ui
