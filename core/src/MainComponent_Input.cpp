#include "MainComponent.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

#include "AudioEngine.h"
#include "MainComponentHelpers.h"
#include "MainComponent_ModulePanelEnvelope.h"
#include "core/AudioModules.h"

using soundtable::ui::isConnectionGeometricallyActive;
using soundtable::ui::makeConnectionKey;

void MainComponent::mouseMove(const juce::MouseEvent& event)
{
    // Do not trigger repaints on plain hover events. All visual
    // updates are driven by the timer and by explicit drag/click
    // gestures (mouseDown/mouseDrag/mouseUp/mouseWheelMove).
    juce::Component::mouseMove(event);
}

void MainComponent::mouseDoubleClick(const juce::MouseEvent& event)
{
    // Ignore right-button double-clicks so that only left-button
    // double-clicks open the per-module detail panel.
    if (event.mods.isRightButtonDown()) {
        juce::Component::mouseDoubleClick(event);
        return;
    }

    const auto bounds = getLocalBounds().toFloat();
    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    const juce::Point<float> pos = event.position;
    constexpr float kHitRadius = 30.0F;

    for (const auto& [id, object] : objects) {
        juce::ignoreUnused(id);

        if (object.docked() ||
            object.logical_id() == soundtable::MASTER_OUTPUT_ID) {
            continue;
        }

        if (!isInsideMusicArea(object)) {
            continue;
        }

        const auto centrePos = objectTableToScreen(object, bounds);
        const float cx = centrePos.x;
        const float cy = centrePos.y;

        const float dx = pos.x - cx;
        const float dy = pos.y - cy;
        const float distSq = dx * dx + dy * dy;
        if (distSq > kHitRadius * kHitRadius) {
            continue;
        }

        // Open or focus the per-module detail panel for the module
        // under the double-click. The window remains open until the
        // user explicitly clicks the close tab.
        ModulePanelState panel{};
        panel.moduleId = object.logical_id();
        panel.visible = true;

        // Default to the first settings tab declared by the module.
        // This allows each module to decide whether the initial view
        // should be Envelope, Files, Settings, etc.
        panel.activeTab = ModulePanelState::Tab::kSettings;

        const auto modIt = modules.find(panel.moduleId);
        if (modIt != modules.end() && modIt->second != nullptr) {
            auto* modulePtr = modIt->second.get();
            const auto& tabs = modulePtr->supported_settings_tabs();
            if (!tabs.empty()) {
                using Kind = soundtable::AudioModule::SettingsTabKind;
                const Kind kind = tabs.front().kind;
                switch (kind) {
                case Kind::kEnvelope:
                    panel.activeTab = ModulePanelState::Tab::kEnvelope;
                    break;
                case Kind::kLoopFiles:
                    panel.activeTab = ModulePanelState::Tab::kLoopFiles;
                    break;
                case Kind::kXYControl:
                    panel.activeTab = ModulePanelState::Tab::kXYControl;
                    break;
                case Kind::kSettings:
                default:
                    panel.activeTab = ModulePanelState::Tab::kSettings;
                    break;
                }
            }
        }

        modulePanels_[panel.moduleId] = panel;
        repaintWithRateLimit();
        return;
    }

    // Fallback to default behaviour when double-clicking outside
    // any module.
    juce::Component::mouseDoubleClick(event);
}

void MainComponent::mouseWheelMove(const juce::MouseEvent& event,
                                   const juce::MouseWheelDetails& wheel)
{
    // Allow the JUCE base class to handle any default behaviour
    // (scrolling in parent components, etc.) before applying our own
    // interaction mapping.
    juce::Component::mouseWheelMove(event, wheel);

    soundtable::ui::InteractionSystem::WheelEvent normalised{};
    normalised.position = event.position;
    normalised.deltaY = static_cast<float>(wheel.deltaY);
    normalised.isCtrlDown = event.mods.isCtrlDown();
    normalised.isShiftDown = event.mods.isShiftDown();
    normalised.source = soundtable::ui::InteractionSystem::WheelEvent::Source::kMouse;

    interactionSystem_.handleWheel(normalised);
}

void MainComponent::mouseDown(const juce::MouseEvent& event)
{
    soundtable::ui::InteractionSystem::PointerEvent normalised{};
    normalised.position = event.position;
    normalised.isPrimary = true;
    normalised.isRightButton = event.mods.isRightButtonDown();
    normalised.isCtrlDown = event.mods.isCtrlDown();
    normalised.isShiftDown = event.mods.isShiftDown();
    normalised.source = soundtable::ui::InteractionSystem::PointerEvent::Source::kMouse;

    interactionSystem_.handlePointerDown(normalised);
}

void MainComponent::handlePointerDown(juce::Point<float> position,
                                      const juce::ModifierKeys& mods)
{
    soundtable::ui::InteractionSystem::PointerEvent event{};
    event.position = position;
    event.isPrimary = true;
    event.isRightButton = mods.isRightButtonDown();
    event.isCtrlDown = mods.isCtrlDown();
    event.isShiftDown = mods.isShiftDown();
    event.source = soundtable::ui::InteractionSystem::PointerEvent::Source::kMouse;

    interactionSystem_.handlePointerDown(event);
}

void MainComponent::mouseDrag(const juce::MouseEvent& event)
{
    soundtable::ui::InteractionSystem::PointerEvent normalised{};
    normalised.position = event.position;
    normalised.isPrimary = true;
    normalised.isRightButton = event.mods.isRightButtonDown();
    normalised.isCtrlDown = event.mods.isCtrlDown();
    normalised.isShiftDown = event.mods.isShiftDown();
    normalised.source = soundtable::ui::InteractionSystem::PointerEvent::Source::kMouse;

    interactionSystem_.handlePointerDrag(normalised);
}

void MainComponent::handlePointerDrag(juce::Point<float> position,
                                      const juce::ModifierKeys& mods)
{
    soundtable::ui::InteractionSystem::PointerEvent event{};
    event.position = position;
    event.isPrimary = true;
    event.isRightButton = mods.isRightButtonDown();
    event.isCtrlDown = mods.isCtrlDown();
    event.isShiftDown = mods.isShiftDown();
    event.source = soundtable::ui::InteractionSystem::PointerEvent::Source::kMouse;

    interactionSystem_.handlePointerDrag(event);
}

void MainComponent::maybeRetriggerOscillatorOnFreqChange(
    const std::string& moduleId, soundtable::AudioModule* module)
{
    // Only Oscillator modules expose a per-voice amplitude envelope
    // that we can explicitly retrigger on pitch/frequency changes.
    if (module == nullptr ||
        !module->is<soundtable::OscillatorModule>()) {
        return;
    }

    const auto it = moduleVoiceIndex_.find(moduleId);
    if (it == moduleVoiceIndex_.end()) {
        return;
    }

    const int voiceIndex = it->second;
    if (voiceIndex < 0 || voiceIndex >= AudioEngine::kMaxVoices) {
        return;
    }

    // Treat any effective pitch/frequency change as a new MIDI-style
    // note event for this Oscillator voice so that the envelope
    // restarts from its attack phase and the updated ADSR settings
    // are always applied, regardless of whether the visual envelope
    // has a sustain plateau or behaves like a one-shot shape.
    audioEngine_.triggerVoiceEnvelope(voiceIndex);
}

void MainComponent::updateOscillatorSustainFromEnvelope(
    const std::string& moduleId)
{
    const auto& modules = scene_.modules();
    const auto modIt = modules.find(moduleId);
    if (modIt == modules.end() || modIt->second == nullptr) {
        return;
    }

    auto* module = modIt->second.get();
    auto* oscModule = dynamic_cast<soundtable::OscillatorModule*>(module);
    if (oscModule == nullptr) {
        return;
    }

    const auto& env = oscModule->envelope();
    const auto& xs = env.points_x;
    const auto& ys = env.points_y;
    const std::size_t n = ys.size();

    float sustainLevel = 1.0F;
    bool hasSustainPlateau = false;

    if (n > 0 && xs.size() == n) {
        constexpr float kMinPlateauWidth = 0.15F;

        float bestLevel = 1.0F;
        float bestWidth = 0.0F;

        for (std::size_t i = 1; i + 1 < n;) {
            const float y = ys[i];
            if (y <= 0.0F) {
                ++i;
                continue;
            }

            std::size_t j = i + 1;
            while (j + 1 < n && std::abs(ys[j] - y) < 1.0e-3F) {
                ++j;
            }

            const float width =
                static_cast<float>(xs[j - 1] - xs[i]);
            if (width > bestWidth) {
                bestWidth = width;
                bestLevel = y;
            }

            i = j;
        }

        if (bestWidth >= kMinPlateauWidth && bestLevel > 0.0F) {
            sustainLevel = bestLevel;
            hasSustainPlateau = true;
        } else {
            float maxY = 0.0F;
            for (std::size_t i = 0; i + 1 < n; ++i) {
                maxY = std::max(maxY, ys[i]);
            }
            sustainLevel = (maxY > 0.0F) ? maxY : 1.0F;
        }
    }

    const float effectiveReleaseMs =
        (env.release > 0.0F) ? env.release : env.decay;
    const float durationMsToUse =
        hasSustainPlateau ? 0.0F : env.duration;

    const auto voiceIt = moduleVoiceIndex_.find(moduleId);
    if (voiceIt == moduleVoiceIndex_.end()) {
        return;
    }

    const int voiceIndex = voiceIt->second;
    if (voiceIndex < 0 || voiceIndex >= AudioEngine::kMaxVoices) {
        return;
    }

    audioEngine_.setVoiceEnvelope(voiceIndex, env.attack, env.decay,
                                  durationMsToUse, effectiveReleaseMs,
                                  sustainLevel);
}

void MainComponent::mouseUp(const juce::MouseEvent& event)
{
    soundtable::ui::InteractionSystem::PointerEvent normalised{};
    normalised.position = lastPointerPosition_;
    normalised.isPrimary = true;
    normalised.isRightButton = event.mods.isRightButtonDown();
    normalised.isCtrlDown = event.mods.isCtrlDown();
    normalised.isShiftDown = event.mods.isShiftDown();
    normalised.source = soundtable::ui::InteractionSystem::PointerEvent::Source::kMouse;

    interactionSystem_.handlePointerUp(normalised);
}

void MainComponent::handlePointerUp(const juce::ModifierKeys& mods)
{
    juce::ignoreUnused(mods);

    soundtable::ui::InteractionSystem::PointerEvent event{};
    event.position = lastPointerPosition_;
    event.isPrimary = true;
    event.isRightButton = false;
    event.isCtrlDown = false;
    event.isShiftDown = false;
    event.source = soundtable::ui::InteractionSystem::PointerEvent::Source::kMouse;

    interactionSystem_.handlePointerUp(event);
}

void MainComponent::handleTuioCursorDown(const float normX,
                                         const float normY)
{
    const auto bounds = getLocalBounds().toFloat();
    juce::Point<float> position{bounds.getX() + normX * bounds.getWidth(),
                                bounds.getY() + normY * bounds.getHeight()};

    juce::Logger::writeToLog(
        "[soundtable-core] TUIO cursor DOWN at px=" +
        juce::String(position.x, 1) + "," +
        juce::String(position.y, 1));

    soundtable::ui::InteractionSystem::PointerEvent normalised{};
    normalised.position = position;
    normalised.isPrimary = true;
    normalised.isRightButton = false;
    normalised.isCtrlDown = false;
    normalised.isShiftDown = false;
    normalised.source = soundtable::ui::InteractionSystem::PointerEvent::Source::kTuio;

    interactionSystem_.handlePointerDown(normalised);
}

void MainComponent::handleTuioCursorMove(const float normX,
                                         const float normY)
{
    const auto bounds = getLocalBounds().toFloat();
    juce::Point<float> position{bounds.getX() + normX * bounds.getWidth(),
                                bounds.getY() + normY * bounds.getHeight()};

    juce::Logger::writeToLog(
        "[soundtable-core] TUIO cursor MOVE at px=" +
        juce::String(position.x, 1) + "," +
        juce::String(position.y, 1));

    soundtable::ui::InteractionSystem::PointerEvent normalised{};
    normalised.position = position;
    normalised.isPrimary = true;
    normalised.isRightButton = false;
    normalised.isCtrlDown = false;
    normalised.isShiftDown = false;
    normalised.source = soundtable::ui::InteractionSystem::PointerEvent::Source::kTuio;

    interactionSystem_.handlePointerDrag(normalised);
}

void MainComponent::handleTuioCursorUp()
{
    soundtable::ui::InteractionSystem::PointerEvent normalised{};
    normalised.position = lastPointerPosition_;
    normalised.isPrimary = true;
    normalised.isRightButton = false;
    normalised.isCtrlDown = false;
    normalised.isShiftDown = false;
    normalised.source = soundtable::ui::InteractionSystem::PointerEvent::Source::kTuio;

    interactionSystem_.handlePointerUp(normalised);
}

void MainComponent::applyControlDropMuteIfNeeded(
    const juce::ModifierKeys& mods)
{
    if (!mods.isCtrlDown()) {
        return;
    }

    if (draggedObjectId_ == 0) {
        return;
    }

    const auto& objects = scene_.objects();
    const auto it = objects.find(draggedObjectId_);
    if (it == objects.end()) {
        return;
    }

    const auto& obj = it->second;
    if (!isInsideMusicArea(obj)) {
        return;
    }

    const auto& modules = scene_.modules();
    const auto modIt = modules.find(obj.logical_id());
    if (modIt == modules.end() || modIt->second == nullptr) {
        return;
    }

    auto* module = modIt->second.get();

    // Only apply this behaviour to Loop, Oscillator and Sampleplay
    // modules, mapping to their respective gain/amp parameters.
    if (module->is<soundtable::OscillatorModule>()) {
        scene_.SetModuleParameter(obj.logical_id(), "gain", 0.0F);
    } else if (module->is<soundtable::LoopModule>() ||
               module->is<soundtable::SampleplayModule>()) {
        scene_.SetModuleParameter(obj.logical_id(), "amp", 0.0F);
    }
}
