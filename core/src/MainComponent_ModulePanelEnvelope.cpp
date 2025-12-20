#include "MainComponent_ModulePanelEnvelope.h"

ModuleEnvelopeGeometry computeModuleEnvelopeGeometry(
    const juce::Rectangle<float>& panelBounds)
{
    ModuleEnvelopeGeometry geom{};

    // No extra padding: use the full inner panel rectangle.
    geom.contentBounds = panelBounds;

    constexpr float kBottomBarHeight = 22.0F;
    juce::Rectangle<float> envelopeArea = geom.contentBounds;
    geom.bottomBar = envelopeArea.removeFromBottom(kBottomBarHeight);
    geom.barsArea = envelopeArea;

    return geom;
}

void paintModuleEnvelopeView(
    juce::Graphics& g,
    const juce::Rectangle<float>& panelBounds,
    const rectai::AudioModuleWithEnvelope& module,
    const bool atlasLoaded,
    const std::function<juce::Image(const std::string&, int, int)>&
        getCachedIcon,
    const float attackMaxMs,
    const float decayMaxMs,
    const float durationMaxMs,
    const float releaseMaxMs)
{
    const auto geom = computeModuleEnvelopeGeometry(panelBounds);
    const auto& env = module.envelope();

    // Bars area: 4 vertical bars A, D, S (duration), R.
    juce::Rectangle<float> barsArea = geom.barsArea;

    const int numBars = 4;
    const float spacing = 1.0F;  // 1 px gap between internal bars.
    const float totalSpacing =
        spacing * static_cast<float>(numBars - 1);
    const float rawWidth = barsArea.getWidth() - totalSpacing;
    if (rawWidth <= 0.0F) {
        return;
    }

    const float barWidth = rawWidth / static_cast<float>(numBars);

    const float attack01 = juce::jlimit(
        0.0F, 1.0F, attackMaxMs > 0.0F ? env.attack / attackMaxMs
                                       : 0.0F);
    const float decay01 = juce::jlimit(
        0.0F, 1.0F, decayMaxMs > 0.0F ? env.decay / decayMaxMs : 0.0F);
    const float release01 = juce::jlimit(
        0.0F, 1.0F, releaseMaxMs > 0.0F ? env.release / releaseMaxMs
                                         : 0.0F);

    const char* labels[4] = {"A", "D", "S", "R"};

    for (int i = 0; i < numBars; ++i) {
        const float x = barsArea.getX() +
                        (barWidth + spacing) *
                            static_cast<float>(i);
        juce::Rectangle<float> barBounds(
            x, barsArea.getY(), barWidth, barsArea.getHeight());

        // Background track.
        g.setColour(juce::Colours::white.withAlpha(0.10F));
        g.fillRoundedRectangle(barBounds, 2.0F);

        float vVisual = 0.0F;
        if (i == 0) {
            // Attack time: non-linear visual scaling from attackMs.
            const float vLinear = juce::jlimit(0.0F, 1.0F, attack01);
            vVisual = moduleEnvelopeVisualFromNormalised(vLinear);
        } else if (i == 1) {
            // Decay time: non-linear visual scaling from decayMs.
            const float vLinear = juce::jlimit(0.0F, 1.0F, decay01);
            vVisual = moduleEnvelopeVisualFromNormalised(vLinear);
        } else if (i == 2) {
            // Sustain level: visualised linearly from the third
            // control point in points_y (index 2), interpreted as
            // a normalised 0..1 sustain value.
            float sustain01 = 1.0F;
            const auto& ys = env.points_y;
            if (ys.size() > 2) {
                sustain01 = ys[2];
            }
            vVisual = juce::jlimit(0.0F, 1.0F, sustain01);
        } else {
            // Release time: non-linear visual scaling from releaseMs.
            const float vLinear = juce::jlimit(0.0F, 1.0F, release01);
            vVisual = moduleEnvelopeVisualFromNormalised(vLinear);
        }

        if (vVisual > 0.0F) {
            const float filledHeight = barBounds.getHeight() * vVisual;
            juce::Rectangle<float> fillBounds = barBounds;
            const float filledY = barBounds.getBottom() - filledHeight;
            fillBounds.setY(filledY);
            fillBounds.setHeight(filledHeight);
            g.setColour(juce::Colours::white.withAlpha(0.85F));
            g.fillRoundedRectangle(fillBounds, 2.0F);
        }

        // Label centred at the bottom of each bar.
        g.setColour(juce::Colours::black.withAlpha(0.80F));
        g.setFont(juce::Font(10.0F));
        g.drawFittedText(labels[i], barBounds.toNearestInt(),
                         juce::Justification::centredBottom, 1);
    }

    // Bottom preset buttons: square, icon-only, one on the left
    // (tab_erase) and one on the right (tab_envelope). No explicit
    // background fill, only the atlas icon.
    const float buttonSide = geom.bottomBar.getHeight();
    juce::Rectangle<float> leftButton(
        geom.bottomBar.getX(), geom.bottomBar.getY(), buttonSide,
        geom.bottomBar.getHeight());
    juce::Rectangle<float> rightButton(
        geom.bottomBar.getRight() - buttonSide, geom.bottomBar.getY(),
        buttonSide, geom.bottomBar.getHeight());

    auto drawPresetIcon = [&](const juce::Rectangle<float>& b,
                              const std::string& iconId) {
        const int destSize =
            static_cast<int>(std::floor(b.getHeight()));
        if (destSize <= 0 || !atlasLoaded) {
            return;
        }

        auto iconImage = getCachedIcon(iconId, destSize, destSize);
        if (!iconImage.isValid()) {
            return;
        }

        const int destX = juce::roundToInt(b.getX());
        const int destY = juce::roundToInt(b.getY());
        g.setColour(juce::Colours::white.withAlpha(0.90F));
        g.drawImageAt(iconImage, destX, destY);
    };

    drawPresetIcon(leftButton, "tab_erase");
    drawPresetIcon(rightButton, "tab_envelope");
}

ModuleEnvelopeHitResult hitTestModuleEnvelope(
    const juce::Rectangle<float>& panelBounds,
    const juce::Point<float> localPos,
    const float /*attackMaxMs*/, const float /*decayMaxMs*/,
    const float /*durationMaxMs*/, const float /*releaseMaxMs*/)
{
    ModuleEnvelopeHitResult result{};

    const auto geom = computeModuleEnvelopeGeometry(panelBounds);

    // Bottom buttons.
    const float buttonSide = geom.bottomBar.getHeight();
    juce::Rectangle<float> leftButton(
        geom.bottomBar.getX(), geom.bottomBar.getY(), buttonSide,
        geom.bottomBar.getHeight());
    juce::Rectangle<float> rightButton(
        geom.bottomBar.getRight() - buttonSide, geom.bottomBar.getY(),
        buttonSide, geom.bottomBar.getHeight());

    if (leftButton.contains(localPos)) {
        result.kind = ModuleEnvelopeHitKind::kPresetLeft;
        return result;
    }
    if (rightButton.contains(localPos)) {
        result.kind = ModuleEnvelopeHitKind::kPresetRight;
        return result;
    }

    // Bars area.
    if (!geom.barsArea.contains(localPos)) {
        return result;
    }

    const int numBars = 4;
    const float spacing = 1.0F;
    const float totalSpacing =
        spacing * static_cast<float>(numBars - 1);
    const float rawWidth = geom.barsArea.getWidth() - totalSpacing;
    if (rawWidth <= 0.0F) {
        return result;
    }

    const float barWidth = rawWidth / static_cast<float>(numBars);

    const float valueRangeTop = geom.barsArea.getY();
    const float valueRangeBottom = geom.barsArea.getBottom();
    const float clampedY = juce::jlimit(valueRangeTop, valueRangeBottom,
                                        localPos.y);
    const float value01 = juce::jmap(clampedY, valueRangeBottom,
                                     valueRangeTop, 0.0F, 1.0F);

    for (int i = 0; i < numBars; ++i) {
        const float x = geom.barsArea.getX() +
                        (barWidth + spacing) *
                            static_cast<float>(i);
        juce::Rectangle<float> barBounds(
            x, geom.barsArea.getY(), barWidth,
            geom.barsArea.getHeight());
        if (!barBounds.contains(localPos)) {
            continue;
        }

        result.kind = ModuleEnvelopeHitKind::kBar;
        result.barIndex = i;
        result.value01 = juce::jlimit(0.0F, 1.0F, value01);
        return result;
    }

    return result;
}
