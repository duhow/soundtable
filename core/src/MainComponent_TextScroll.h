#pragma once

#include <functional>
#include <vector>

#include <juce_gui_basics/juce_gui_basics.h>

namespace soundtable::ui {

// Generic vertically scrollable text list component used for module
// detail panels (e.g. Loop sample selector). The component displays up
// to a fixed number of visible rows and allows scrolling via the mouse
// wheel. Callers provide the list of items and are notified when the
// selection changes.
class TextScrollList : public juce::Component {
public:
    struct ItemStyle {
        bool bold{false};
        bool useCustomColour{false};
        juce::Colour colour{};
    };

    struct Item {
        juce::String text;
        ItemStyle style{};
    };

    TextScrollList();
    ~TextScrollList() override = default;

    void paint(juce::Graphics& g) override;
    void mouseDown(const juce::MouseEvent& event) override;
    void mouseDrag(const juce::MouseEvent& event) override;
    void mouseUp(const juce::MouseEvent& event) override;
    void mouseWheelMove(const juce::MouseEvent& event,
                        const juce::MouseWheelDetails& wheel) override;

    // Items -----------------------------------------------------------------

    void setItems(const std::vector<Item>& items);
    void setItems(std::vector<Item>&& items);
    void clearItems();

    [[nodiscard]] int getNumItems() const noexcept;
    [[nodiscard]] const Item* getItem(int index) const noexcept;

    // Selection -------------------------------------------------------------

    void setSelectedIndex(int index, bool notify = true);
    [[nodiscard]] int getSelectedIndex() const noexcept;

    void setOnSelectionChanged(std::function<void(int)> callback);

    // Layout / behaviour ----------------------------------------------------

    void setMaxVisibleItems(int maxItems);
    [[nodiscard]] int getMaxVisibleItems() const noexcept;

    void setRowHeight(float height);
    [[nodiscard]] float getRowHeight() const noexcept;

    void scrollToTop();
    void scrollToBottom();

    // Direct interaction helpers used by owners that embed the
    // TextScrollList into a larger custom-painted surface instead of
    // adding it as a child Component. Coordinates are expressed in
    // the list's local space, with (0,0) at the top-left corner.
    // Full click helper: presiona y suelta en la misma posición.
    void handleClickAt(float x, float y);
    void handleWheelDelta(float deltaY);

    // Fine-grained pointer helpers mirroring mouseDown/Drag/Up para
    // integraciones externas (por ejemplo, routers de input de
    // MainComponent). Las coordenadas están en espacio local.
    void beginPointerAt(float y);
    void dragPointerTo(float y);
    void endPointerAt(float y);

    [[nodiscard]] int indexAtPosition(float y) const noexcept;

private:
    void updateScrollRange();
    void setScrollOffset(float newOffset);

    std::vector<Item> items_;
    int selectedIndex_{-1};
    int maxVisibleItems_{6};
    float rowHeight_{18.0F};
    float scrollOffset_{0.0F};
    float maxScrollOffset_{0.0F};

    // Drag-to-scroll state.
    bool isDragging_{false};
    float dragStartY_{0.0F};
    float dragStartScrollOffset_{0.0F};

    // Click-to-select state: index under the pointer at mouseDown
    // in the current scroll position. Selection is only committed on
    // mouseUp if the pointer is still over the same item.
    int pressedIndex_{-1};

    std::function<void(int)> onSelectionChanged_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(TextScrollList)
};

}  // namespace soundtable::ui
