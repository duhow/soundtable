#pragma once

#include <juce_gui_extra/juce_gui_extra.h>

class MainComponent;

namespace rectai::ui {

// Lightweight interaction façade that receives normalised pointer and
// wheel events from MainComponent (mouse + TUIO) and forwards them into
// the shared interaction logic. This keeps JUCE-specific event types at
// the edges and allows future alternative input sources to reuse the
// same model.
class InteractionSystem {
public:
    struct PointerEvent {
        juce::Point<float> position{};
        bool isPrimary{true};
        bool isRightButton{false};
        bool isCtrlDown{false};
        bool isShiftDown{false};

        enum class Source { kMouse = 0, kTuio = 1 };
        Source source{Source::kMouse};
    };

    struct WheelEvent {
        juce::Point<float> position{};
        float deltaY{0.0F};
        bool isCtrlDown{false};
        bool isShiftDown{false};

        enum class Source { kMouse = 0, kTuio = 1 };
        Source source{Source::kMouse};
    };

    explicit InteractionSystem(MainComponent& owner) noexcept;

    void handlePointerDown(const PointerEvent& event);
    void handlePointerDrag(const PointerEvent& event);
    void handlePointerUp(const PointerEvent& event);

    void handleWheel(const WheelEvent& event);

private:
    MainComponent& owner_;
};

}  // namespace rectai::ui
