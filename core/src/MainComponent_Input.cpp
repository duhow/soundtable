#include "MainComponent.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

#include "MainComponentHelpers.h"
#include "core/AudioModules.h"

using rectai::ui::isConnectionGeometricallyActive;
using rectai::ui::makeConnectionKey;

void MainComponent::mouseWheelMove(const juce::MouseEvent& event,
                                   const juce::MouseWheelDetails& wheel)
{
    const auto bounds = getLocalBounds().toFloat();

    // When the cursor is over the Tempo module, use the mouse wheel
    // to adjust the global BPM in 1-unit steps instead of scrolling
    // the dock. This provides a quick tempo nudge gesture that
    // complements rotation and the side control bar.
    {
        const auto& objects = scene_.objects();
        const auto& modules = scene_.modules();

        const float tempoRadius = 30.0F;
        for (const auto& [id, object] : objects) {
            juce::ignoreUnused(id);

            const auto modIt = modules.find(object.logical_id());
            if (modIt == modules.end() || modIt->second == nullptr) {
                continue;
            }

            auto* module = modIt->second.get();
            auto* tempoModule =
                dynamic_cast<rectai::TempoModule*>(module);
            if (tempoModule == nullptr) {
                continue;
            }

            if (object.docked()) {
                // For now, only support wheel-based tempo changes
                // when the Tempo tangible is on the table surface.
                continue;
            }

            const float cx = bounds.getX() +
                             object.x() * bounds.getWidth();
            const float cy = bounds.getY() +
                             object.y() * bounds.getHeight();

            const float dx = static_cast<float>(event.position.x) - cx;
            const float dy = static_cast<float>(event.position.y) - cy;
            const float distSq = dx * dx + dy * dy;
            if (distSq > tempoRadius * tempoRadius) {
                continue;
            }

            const float wheelDelta = static_cast<float>(wheel.deltaY);
            if (wheelDelta == 0.0F) {
                return;
            }

            const double baseStep = event.mods.isShiftDown() ? 5.0 : 1.0;
            const double step = (wheelDelta > 0.0F) ? baseStep : -baseStep;
            double newBpm = bpm_ + step;
            newBpm = juce::jlimit(40.0, 400.0, newBpm);

            if (newBpm == bpm_) {
                return;
            }

            bpm_ = newBpm;
            bpmLastChangeSeconds_ =
                juce::Time::getMillisecondCounterHiRes() / 1000.0;

            // Keep the logical Tempo module parameter in sync with
            // the global BPM so that serialisation and other
            // consumers observe the updated tempo value.
            scene_.SetModuleParameter(tempoModule->id(), "tempo",
                                      static_cast<float>(bpm_));

            repaint();
            return;
        }
    }

    // Limit mouse wheel scrolling to the dock area only.
    auto dockBounds = bounds;
    const float dockWidth = calculateDockWidth(dockBounds.getWidth());
    juce::Rectangle<float> dockArea =
        dockBounds.removeFromRight(dockWidth);

    if (!dockArea.contains(event.position)) {
        juce::Component::mouseWheelMove(event, wheel);
        return;
    }

    const auto& objects = scene_.objects();
    std::size_t dockCount = 0;
    for (const auto& [id, obj] : objects) {
        juce::ignoreUnused(id);
        if (obj.docked()) {
            ++dockCount;
        }
    }

    if (dockCount == 0) {
        return;
    }

    // Replicate the same boundary logic used when dragging the dock.
    const float titleHeight = 24.0F;
    juce::Rectangle<float> titleArea =
        dockArea.removeFromTop(titleHeight);
    juce::ignoreUnused(titleArea);

    const float availableHeight = dockArea.getHeight();
    const float nodeRadiusDock = 18.0F;
    const float verticalPadding = 12.0F;
    const float slotHeight =
        nodeRadiusDock * 2.0F + verticalPadding;
    const float contentHeight =
        slotHeight * static_cast<float>(dockCount);
    const float minOffset =
        (contentHeight > availableHeight)
            ? (availableHeight - contentHeight)
            : 0.0F;

    // deltaY > 0 means wheel up; scroll content in steps proportional
    // to the height of a single dock slot so that each wheel notch
    // advances approximately one module.
    const float scrollStepPixels = slotHeight;
    const float delta = static_cast<float>(wheel.deltaY) *
                        scrollStepPixels;

    dockScrollOffset_ = juce::jlimit(minOffset, 0.0F,
                                     dockScrollOffset_ + delta);
    repaint();
}

void MainComponent::mouseDown(const juce::MouseEvent& event)
{
    const auto bounds = getLocalBounds().toFloat();
    const float radius = 30.0F;

    const bool isRightClick = event.mods.isRightButtonDown();

    // Track touch interface state for visual feedback.
    isTouchActive_ = true;
    isTouchHeld_ = false;
    currentTouchPosition_ = event.position;
    touchTrail_.clear();
    touchCutConnections_.clear();
    touchCutObjects_.clear();
    touchCurrentlyIntersectingConnections_.clear();
    touchCurrentlyIntersectingObjects_.clear();
    // Reset cut mode; it will only be enabled explicitly at the end
    // of this handler if the pointer started on empty musical space
    // (no module, no line, no dock interaction).
    isCutModeActive_ = false;

    // Determine if touch started in dock area.
    const float dockWidthMain = calculateDockWidth(bounds.getWidth());
    auto boundsCopy = bounds;
    juce::Rectangle<float> dockAreaMain =
        boundsCopy.removeFromRight(dockWidthMain);
    touchStartedInDock_ = dockAreaMain.contains(event.position);

    repaint();

    draggedObjectId_ = 0;
    sideControlObjectId_ = 0;
    sideControlKind_ = SideControlKind::kNone;

    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    // Right-click on an Oscillator tangible cycles its waveform
    // (sine -> saw -> square -> noise -> ...), updating the icon
    // to match the selected subtype. Right-click on a Filter tangible
    // cycles its mode (low-pass -> band-pass -> high-pass) and updates
    // the icon accordingly. Right-click on a Sampleplay tangible
    // cycles through the available instruments declared in the
    // Reactable patch, updating the title rendered next to the
    // module.
    if (isRightClick) {
        for (const auto& [id, object] : objects) {
            if (object.logical_id() == rectai::MASTER_OUTPUT_ID ||
                object.docked()) {
                continue;
            }

            const auto cx = bounds.getX() +
                            object.x() * bounds.getWidth();
            const auto cy = bounds.getY() +
                            object.y() * bounds.getHeight();

            const auto dx = static_cast<float>(event.position.x) - cx;
            const auto dy = static_cast<float>(event.position.y) - cy;
            const auto distanceSquared = dx * dx + dy * dy;

            if (distanceSquared > radius * radius) {
                continue;
            }

            const auto modIt = modules.find(object.logical_id());
            if (modIt == modules.end() ||
                modIt->second == nullptr) {
                continue;
            }

            auto* oscModule = dynamic_cast<rectai::OscillatorModule*>(
                modIt->second.get());
            if (oscModule != nullptr) {
                oscModule->cycle_waveform();
                repaint();
                return;
            }

            auto* filterModule =
                dynamic_cast<rectai::FilterModule*>(modIt->second.get());
            if (filterModule != nullptr) {
                filterModule->cycle_mode();
                repaint();
                return;
            }

            if (auto* sampleModule =
                    dynamic_cast<rectai::SampleplayModule*>(
                        modIt->second.get())) {
                // Right-click on Sampleplay cycles instruments within
                // the current logical bank. Holding CTRL while
                // right-clicking switches to the next logical bank
                // (e.g. drums ↔ synth) using the default preset
                // associated with that bank.
                if (event.mods.isCtrlDown()) {
                    sampleModule->CycleBank();
                } else {
                    sampleModule->CycleInstrument();
                }
                markSampleplayInstrumentLabelActive(sampleModule->id());
                repaint();
                return;
            }
        }
        // Right-click that does not hit any oscillator or filter tangible
        // falls through without side effects.
        return;
    }

    // First, try to grab one of the per-instrument side controls
    // (left: Freq, right: Gain). Clicking anywhere on the control
    // bar (not solo en el handle) moves the value to that position
    // and starts a drag gesture from there.
    const float nodeRadius = 26.0F;
    const float ringRadius = nodeRadius + 10.0F;
    const float sliderMargin = 6.0F;

    for (const auto& [id, object] : objects) {
        if (object.logical_id() == rectai::MASTER_OUTPUT_ID) {
            continue;
        }

        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        const bool insideMusic = isInsideMusicArea(object);
        float rotationAngle = 0.0F;
        if (insideMusic) {
            const float dx = cx - bounds.getCentreX();
            const float dy = cy - bounds.getCentreY();
            rotationAngle =
                std::atan2(dy, dx) -
                juce::MathConstants<float>::halfPi;
        }

        const auto modIt = modules.find(object.logical_id());
        const rectai::AudioModule* moduleForObject =
            (modIt != modules.end()) ? modIt->second.get() : nullptr;
        const bool isTempoModule =
            (moduleForObject != nullptr &&
             moduleForObject->is<rectai::TempoModule>());
        bool freqEnabled = false;
        bool gainEnabled = false;
        float freqValue = 0.5F;
        float gainValue = 0.5F;
        if (moduleForObject != nullptr) {
            if (isTempoModule) {
                const double minBpm = 40.0;
                const double maxBpm = 400.0;
                const double clampedBpm =
                    juce::jlimit(minBpm, maxBpm, bpm_);
                const double norm =
                    (clampedBpm - minBpm) / (maxBpm - minBpm);
                freqValue = static_cast<float>(norm);
            } else {
                freqValue = moduleForObject->GetParameterOrDefault(
                    "freq",
                    moduleForObject->default_parameter_value("freq"));
            }

            gainEnabled = moduleForObject->uses_gain_control();

            if (const auto* volumeModule =
                    dynamic_cast<const rectai::VolumeModule*>(
                        moduleForObject)) {
                gainValue = volumeModule->GetParameterOrDefault(
                    "volume",
                    volumeModule->default_parameter_value("volume"));
            } else if (moduleForObject->type() ==
                       rectai::ModuleType::kFilter) {
                gainValue = moduleForObject->GetParameterOrDefault(
                    "q",
                    moduleForObject->default_parameter_value("q"));
            } else {
                gainValue = moduleForObject->GetParameterOrDefault(
                    "gain",
                    moduleForObject->default_parameter_value("gain"));
            }

            freqEnabled =
                moduleForObject->uses_frequency_control() ||
                isTempoModule;
        }
        freqValue = juce::jlimit(0.0F, 1.0F, freqValue);
        gainValue = juce::jlimit(0.0F, 1.0F, gainValue);

        const float sliderTop = cy - ringRadius + sliderMargin;
        const float sliderBottom = cy + ringRadius - sliderMargin;

        const float freqHandleY = juce::jmap(freqValue, 0.0F, 1.0F,
                                             sliderBottom, sliderTop);
        const float gainHandleY = juce::jmap(gainValue, 0.0F, 1.0F,
                                             sliderBottom, sliderTop);

        const float freqDy = freqHandleY - cy;
        const float gainDy = gainHandleY - cy;
        const float freqInside = ringRadius * ringRadius - freqDy * freqDy;
        const float gainInside = ringRadius * ringRadius - gainDy * gainDy;

        float freqHandleX = cx - ringRadius;
        float gainHandleX = cx + ringRadius;
        if (freqInside >= 0.0F) {
            const float dx = std::sqrt(freqInside);
            freqHandleX = cx - dx;
        }
        if (gainInside >= 0.0F) {
            const float dx = std::sqrt(gainInside);
            gainHandleX = cx + dx;
        }

        juce::Point<float> click = event.position;
        if (insideMusic) {
            click = click.transformedBy(
                juce::AffineTransform::rotation(-rotationAngle, cx, cy));
        }

        auto isOnFreqControlBar = [sliderTop, sliderBottom](float hx,
                                                            juce::Point<float> p) {
            // Treat the side control as a vertical bar that extends
            // further towards the outside of the table (screen left),
            // while keeping the inner edge close to the node so that
            // the centre area remains available to grab the module.
            const float halfWidthRight = 14.0F;
            const float extraLeft = 7.0F;
            const float leftBoundary = hx - (halfWidthRight + extraLeft);
            const float rightBoundary = hx + halfWidthRight;
            const bool withinX = (p.x >= leftBoundary && p.x <= rightBoundary);
            const bool withinY = (p.y >= sliderTop && p.y <= sliderBottom);
            return withinX && withinY;
        };

        auto isOnGainControlBar = [sliderTop, sliderBottom](float hx,
                                                            juce::Point<float> p) {
            // Extend the clickable area slightly further towards the
            // outside of the table (screen right) while keeping the
            // inner edge close to the node so that the centre area
            // remains available to grab the module.
            const float halfWidthLeft = 14.0F;
            const float extraRight = 7.0F;
            const float leftBoundary = hx - halfWidthLeft;
            const float rightBoundary = hx + halfWidthLeft + extraRight;
            const bool withinX = (p.x >= leftBoundary && p.x <= rightBoundary);
            const bool withinY = (p.y >= sliderTop && p.y <= sliderBottom);
            return withinX && withinY;
        };

        // Clicking anywhere on the frequency bar.
        if (freqEnabled && isOnFreqControlBar(freqHandleX, click)) {
            sideControlObjectId_ = id;
            sideControlKind_ = SideControlKind::kFreq;

            const float clampedY = juce::jlimit(sliderTop, sliderBottom, click.y);
            const float value = juce::jmap(clampedY, sliderBottom, sliderTop,
                                           0.0F, 1.0F);

            if (isTempoModule) {
                const double minBpm = 40.0;
                const double maxBpm = 400.0;
                const double newBpm =
                    minBpm + (maxBpm - minBpm) *
                                 static_cast<double>(value);
                bpm_ = juce::jlimit(minBpm, maxBpm, newBpm);

                bpmLastChangeSeconds_ =
                    juce::Time::getMillisecondCounterHiRes() /
                    1000.0;

                scene_.SetModuleParameter(object.logical_id(), "tempo",
                                          static_cast<float>(bpm_));
            } else {
                scene_.SetModuleParameter(object.logical_id(), "freq", value);
            }

            repaint();
            return;
        }

        // Clicking anywhere on the gain bar.
        if (gainEnabled && isOnGainControlBar(gainHandleX, click)) {
            sideControlObjectId_ = id;
            sideControlKind_ = SideControlKind::kGain;

            const float clampedY = juce::jlimit(sliderTop, sliderBottom, click.y);
            const float value = juce::jmap(clampedY, sliderBottom, sliderTop,
                                           0.0F, 1.0F);

            const auto& modulesForGain = scene_.modules();
            const auto modItGain = modulesForGain.find(object.logical_id());
            if (modItGain != modulesForGain.end() &&
                modItGain->second != nullptr) {
                auto* module = modItGain->second.get();
                if (module->type() == rectai::ModuleType::kFilter) {
                    scene_.SetModuleParameter(object.logical_id(), "q",
                                              value);
                } else if (dynamic_cast<rectai::VolumeModule*>(module) !=
                           nullptr) {
                    scene_.SetModuleParameter(object.logical_id(), "volume",
                                              value);
                } else if (dynamic_cast<rectai::LoopModule*>(module) !=
                           nullptr ||
                           dynamic_cast<rectai::SampleplayModule*>(
                               module) != nullptr) {
                    // Loop and Sampleplay expose their main level as
                    // "amp" instead of "gain".
                    scene_.SetModuleParameter(object.logical_id(), "amp",
                                              value);
                } else {
                    scene_.SetModuleParameter(object.logical_id(), "gain",
                                              value);
                }
            } else {
                scene_.SetModuleParameter(object.logical_id(), "gain",
                                          value);
            }

            repaint();
            return;
        }
    }

    // Next, try to select an object by clicking on its circle.
        for (const auto& [id, object] : objects) {
            if (object.logical_id() == rectai::MASTER_OUTPUT_ID ||
                object.docked()) {
            continue;
        }
        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        const auto dx = static_cast<float>(event.position.x) - cx;
        const auto dy = static_cast<float>(event.position.y) - cy;
        const auto distanceSquared = dx * dx + dy * dy;

        if (distanceSquared <= radius * radius) {
            draggedObjectId_ = id;
            break;
        }
    }

    // If no free object was picked, try selecting a docked module from the
    // right-hand dock strip so it can be dragged into the musical area.
    if (draggedObjectId_ == 0) {
        std::vector<std::pair<std::int64_t, const rectai::ObjectInstance*>>
            dockedObjects;
        dockedObjects.reserve(objects.size());
        for (const auto& [id, obj] : objects) {
            if (obj.docked()) {
                dockedObjects.emplace_back(id, &obj);
            }
        }

        if (!dockedObjects.empty()) {
            std::sort(dockedObjects.begin(), dockedObjects.end(),
                      [](const auto& a, const auto& b) {
                          return a.first < b.first;
                      });

            auto dockBounds = bounds;
            const float dockWidthPick =
                calculateDockWidth(dockBounds.getWidth());
            juce::Rectangle<float> dockAreaPick =
                dockBounds.removeFromRight(dockWidthPick);

            const float titleHeight = 24.0F;
            juce::Rectangle<float> titleArea =
                dockAreaPick.removeFromTop(titleHeight);
            juce::ignoreUnused(titleArea);

            const float availableHeight = dockAreaPick.getHeight();
            const float nodeRadiusDock = 18.0F;
            const float verticalPadding = 12.0F;
            const float slotHeight =
                nodeRadiusDock * 2.0F + verticalPadding;
            const float contentHeight =
                slotHeight * static_cast<float>(dockedObjects.size());
            const float minOffset =
                (contentHeight > availableHeight)
                    ? (availableHeight - contentHeight)
                    : 0.0F;
            dockScrollOffset_ = juce::jlimit(minOffset, 0.0F,
                                             dockScrollOffset_);
            const float baseY = dockAreaPick.getY() + dockScrollOffset_;

            for (std::size_t i = 0; i < dockedObjects.size(); ++i) {
                const auto id = dockedObjects[i].first;

                const float cy = baseY + (static_cast<float>(i) + 0.5F) *
                                            slotHeight;
                const float cx = dockAreaPick.getX() +
                                 dockAreaPick.getWidth() * 0.5F;

                const float dx = static_cast<float>(event.position.x) - cx;
                const float dy = static_cast<float>(event.position.y) - cy;
                const float distanceSquared = dx * dx + dy * dy;

                if (distanceSquared <= nodeRadiusDock * nodeRadiusDock) {
                    draggedObjectId_ = id;
                    return;
                }
            }
        }
        // If click is inside the dock but not on any module, start a
        // simple drag-based scroll gesture.
        auto dockBounds = bounds;
        const float dockWidthDrag =
            calculateDockWidth(dockBounds.getWidth());
        juce::Rectangle<float> dockAreaDrag =
            dockBounds.removeFromRight(dockWidthDrag);
        if (dockAreaDrag.contains(event.position)) {
            isDraggingDockScroll_ = true;
            dockLastDragY_ = static_cast<float>(event.position.y);
            return;
        }
    }

    // If no object was selected, check if the click is near a line
    // connecting an object to the centre to toggle that instrument's mute,
    // or near a connection between instruments to toggle the source's mute.
    if (draggedObjectId_ == 0) {
        const auto centre = bounds.getCentre();
        const auto click = event.position;
        constexpr float maxDistance = 6.0F;
        const float maxDistanceSq = maxDistance * maxDistance;

        const auto& objectsLocal = scene_.objects();
        const auto& modulesLocal = scene_.modules();

        auto isPointNearSegment = [](juce::Point<float> p, juce::Point<float> a,
                                     juce::Point<float> b,
                                     const float maxDistSq) {
            const auto ab = b - a;
            const auto ap = p - a;
            const auto abLenSq = ab.x * ab.x + ab.y * ab.y;
            float t = 0.0F;
            if (abLenSq > 0.0F) {
                t = (ap.x * ab.x + ap.y * ab.y) / abLenSq;
                t = juce::jlimit(0.0F, 1.0F, t);
            }
            const auto closest = a + ab * t;
            const auto dx = p.x - closest.x;
            const auto dy = p.y - closest.y;
            const auto distSq = dx * dx + dy * dy;
            return distSq <= maxDistSq;
        };

        // First, allow muting via connections between instruments.
        std::unordered_map<std::string, std::int64_t> moduleToObjectId;
        for (const auto& [id, object] : objectsLocal) {
            moduleToObjectId.emplace(object.logical_id(), id);
        }

        // Track which objects currently have an active outgoing connection
        // so that the hit-test for the centre  object line follows the
        // same visibility rules used in paint(): generators that are feeding
        // another module through an active connection do not render a direct
        // line to the master and therefore should not be clickable either.
        std::unordered_set<std::int64_t> objectsWithOutgoingActiveConnection;
        for (const auto& conn : scene_.connections()) {
            // Ignore connections targeting the invisible Output/master
            // module (id MASTER_OUTPUT_ID) so that auto-wired master
            // links do not affect centre-line hit-testing.
            if (conn.to_module_id == rectai::MASTER_OUTPUT_ID) {
                continue;
            }

            const auto fromIdIt = moduleToObjectId.find(conn.from_module_id);
            const auto toIdIt = moduleToObjectId.find(conn.to_module_id);
            if (fromIdIt == moduleToObjectId.end() ||
                toIdIt == moduleToObjectId.end()) {
                continue;
            }

            const auto fromObjIt = objectsLocal.find(fromIdIt->second);
            const auto toObjIt = objectsLocal.find(toIdIt->second);
            if (fromObjIt == objectsLocal.end() ||
                toObjIt == objectsLocal.end()) {
                continue;
            }

            const auto& fromObj = fromObjIt->second;
            const auto& toObj = toObjIt->second;

            if (!isInsideMusicArea(fromObj) || !isInsideMusicArea(toObj)) {
                continue;
            }

            if (conn.is_hardlink ||
                isConnectionGeometricallyActive(fromObj, toObj)) {
                objectsWithOutgoingActiveConnection.insert(fromIdIt->second);
            }
        }

        for (const auto& conn : scene_.connections()) {
            const auto fromIdIt = moduleToObjectId.find(conn.from_module_id);
            const auto toIdIt = moduleToObjectId.find(conn.to_module_id);
            if (fromIdIt == moduleToObjectId.end() ||
                toIdIt == moduleToObjectId.end()) {
                continue;
            }

            const auto fromObjIt = objectsLocal.find(fromIdIt->second);
            const auto toObjIt = objectsLocal.find(toIdIt->second);
            if (fromObjIt == objectsLocal.end() ||
                toObjIt == objectsLocal.end()) {
                continue;
            }

            const auto& fromObj = fromObjIt->second;
            const auto& toObj = toObjIt->second;

            if (!isInsideMusicArea(fromObj) || !isInsideMusicArea(toObj) ||
                (!conn.is_hardlink &&
                 !isConnectionGeometricallyActive(fromObj, toObj))) {
                continue;
            }

            const auto fx = bounds.getX() + fromObj.x() * bounds.getWidth();
            const auto fy = bounds.getY() + fromObj.y() * bounds.getHeight();
            const auto tx = bounds.getX() + toObj.x() * bounds.getWidth();
            const auto ty = bounds.getY() + toObj.y() * bounds.getHeight();

            if (isPointNearSegment(click, {fx, fy}, {tx, ty}, maxDistanceSq)) {
                const std::string key = makeConnectionKey(conn);

                // Calculate normalized position along the line (0=from, 1=to).
                const juce::Point<float> p1{fx, fy};
                const juce::Point<float> p2{tx, ty};
                const auto ab = p2 - p1;
                const auto ap = click - p1;
                const auto abLenSq = ab.x * ab.x + ab.y * ab.y;
                float splitPoint = 0.5F;  // Default to mid-point.
                if (abLenSq > 0.0F) {
                    splitPoint = (ap.x * ab.x + ap.y * ab.y) / abLenSq;
                    splitPoint = juce::jlimit(0.0F, 1.0F, splitPoint);
                }

                // Store hold state for temporary mute/visualisation.
                // The actual mute is driven by activeConnectionHold_
                // in timerCallback() rather than by mutating
                // mutedConnections_.
                activeConnectionHold_ = ConnectionHoldState{
                    key,
                    0,   // Not an object-to-center line.
                    false,
                    splitPoint};

                repaint();
                return;
            }
        }

        // Fallback: muting via the line centre -> object. Only objects that
        // actually render a visible line to the master in paint() should be
        // clickable here (i.e. only modules that carry audio and not pure
        // control/MIDI modules such as the Sequencer).
        for (const auto& [id, object] : objectsLocal) {
            if (object.logical_id() == rectai::MASTER_OUTPUT_ID ||
                object.docked()) {
                continue;
            }

            if (!isInsideMusicArea(object)) {
                continue;
            }

            const bool hasActiveOutgoingConnection =
                objectsWithOutgoingActiveConnection.find(id) !=
                objectsWithOutgoingActiveConnection.end();

            const auto modForConnectionIt =
                modulesLocal.find(object.logical_id());
            const rectai::AudioModule* moduleForConnection = nullptr;
            if (modForConnectionIt != modulesLocal.end() &&
                modForConnectionIt->second != nullptr) {
                moduleForConnection = modForConnectionIt->second.get();
            }

            const bool isGenerator =
                moduleForConnection != nullptr &&
                moduleForConnection->type() ==
                    rectai::ModuleType::kGenerator;
            const bool isGlobalController =
                moduleForConnection != nullptr &&
                moduleForConnection->is_global_controller();
            const bool isAudioModule =
                moduleForConnection != nullptr &&
                (moduleForConnection->produces_audio() ||
                 moduleForConnection->consumes_audio());

            // Only modules that truly carry audio (produce or consume audio)
            // draw a radial line to the master and therefore should be
            // clickable here. Global controllers and MIDI/control-only
            // modules are excluded.
            if (isGlobalController || !isAudioModule) {
                continue;
            }

            // Generators feeding another module through an active connection
            // hide their direct visual line to the master; that line should
            // not be clickable either.
            if (isGenerator && hasActiveOutgoingConnection) {
                continue;
            }

            const auto cx = bounds.getX() + object.x() * bounds.getWidth();
            const auto cy = bounds.getY() + object.y() * bounds.getHeight();

            if (isPointNearSegment(click, centre, {cx, cy}, maxDistanceSq)) {
                // Calculate normalized position along the line (0=center, 1=object).
                const juce::Point<float> p1 = centre;
                const juce::Point<float> p2{cx, cy};
                const auto ab = p2 - p1;
                const auto ap = click - p1;
                const auto abLenSq = ab.x * ab.x + ab.y * ab.y;
                float splitPoint = 0.5F; // Default to mid-point.
                if (abLenSq > 0.0F) {
                    splitPoint = (ap.x * ab.x + ap.y * ab.y) / abLenSq;
                    splitPoint = juce::jlimit(0.0F, 1.0F, splitPoint);
                }
                
                // Store hold state and apply temporary mute.
                activeConnectionHold_ = ConnectionHoldState{
                    "",  // No connection key for object-to-center lines.
                    id,
                    true,
                    splitPoint
                };

                repaint();
                break;
            }
        }
    }

    // If we reach this point without having started a drag, dock
    // scroll, side control adjustment or hold-mute on a line, and the
    // pointer is inside the musical area (but not in the dock), treat
    // this gesture as a "cut" gesture. This enables the red cursor +
    // trail and line-cut logic during mouseDrag.
    if (!touchStartedInDock_ &&
        draggedObjectId_ == 0 &&
        sideControlObjectId_ == 0 &&
        sideControlKind_ == SideControlKind::kNone &&
        !isDraggingDockScroll_ &&
        !activeConnectionHold_.has_value()) {
        const auto localBounds = getLocalBounds().toFloat();
        const auto centre = localBounds.getCentre();
        const float tableRadius =
            0.45F * std::min(localBounds.getWidth(),
                              localBounds.getHeight());

        const float dx = static_cast<float>(event.position.x) - centre.x;
        const float dy = static_cast<float>(event.position.y) - centre.y;
        const float distSq = dx * dx + dy * dy;

        if (distSq <= tableRadius * tableRadius) {
            isCutModeActive_ = true;
        }
    }
}

void MainComponent::mouseDrag(const juce::MouseEvent& event)
{
    const auto bounds = getLocalBounds().toFloat();

    // Update touch state during drag.
    isTouchHeld_ = true;
    currentTouchPosition_ = event.position;

    // Add point to trail only if touch started in window area (not dock).
    if (!touchStartedInDock_) {
        const double currentTime =
            juce::Time::getMillisecondCounterHiRes() / 1000.0;
        touchTrail_.push_back({event.position, currentTime});

        // Remove old points that have fully faded out and limit trail size.
        if (kEnableTrailFade) {
            touchTrail_.erase(
                std::remove_if(
                    touchTrail_.begin(), touchTrail_.end(),
                    [currentTime](const TrailPoint& point) {
                        return (currentTime - point.timestamp) >=
                               kTrailFadeDurationSeconds;
                    }),
                touchTrail_.end());
        }

        // Hard limit trail size to prevent memory growth.
        if (touchTrail_.size() > static_cast<size_t>(kMaxTrailPoints)) {
            touchTrail_.erase(touchTrail_.begin());
        }

        // Line cutting: detect when the touch trail crosses audio lines
        // (connections or object-to-center lines) and toggle them for
        // mute when the touch is released.
        // This is only enabled in explicit cut mode (red cursor) and is
        // skipped while dragging modules, adjusting side controls or
        // holding a line for temporary mute.
        if (isCutModeActive_ &&
            touchTrail_.size() >= 2 &&
            draggedObjectId_ == 0 &&
            sideControlKind_ == SideControlKind::kNone &&
            !activeConnectionHold_.has_value()) {
            const auto centre = bounds.getCentre();
            const auto& objects = scene_.objects();
            const auto& modules = scene_.modules();

            // Use only the most recent segment with increased threshold
            // for better detection at varying speeds.
            const auto& prevPoint = touchTrail_[touchTrail_.size() - 2];
            const auto& currPoint = touchTrail_[touchTrail_.size() - 1];
            const float detectionThreshold = 15.0F;

            // Check intersections con las líneas objeto-centro. Debe
            // corresponderse con los módulos que realmente dibujan una
            // radial hacia el master en paint(): sólo módulos que
            // producen/consumen audio, excluyendo controladores globales
            // y módulos sólo MIDI/control como el Sequencer.
            for (const auto& [id, object] : objects) {
                if (object.logical_id() == rectai::MASTER_OUTPUT_ID || object.docked() ||
                    !isInsideMusicArea(object)) {
                    continue;
                }

                const auto modIt = modules.find(object.logical_id());
                const rectai::AudioModule* moduleForLine = nullptr;
                if (modIt != modules.end() && modIt->second != nullptr) {
                    moduleForLine = modIt->second.get();
                }

                const bool isGlobalController =
                    moduleForLine != nullptr &&
                    moduleForLine->is_global_controller();
                const bool isAudioModule =
                    moduleForLine != nullptr &&
                    (moduleForLine->produces_audio() ||
                     moduleForLine->consumes_audio());

                // Only modules that truly carry audio (produce or consume
                // audio) have a radial line that can be clicked or cut.
                if (isGlobalController || !isAudioModule) {
                    continue;
                }

                // Skip generator-like modules that are feeding another
                // module. Cualquier módulo no "settings" que tenga una
                // conexión saliente activa oculta su radial directa al
                // master y, por tanto, no debe poder cortarse.
                const bool isGeneratorLike =
                    moduleForLine != nullptr &&
                    moduleForLine->type() !=
                        rectai::ModuleType::kSettings;
                
                bool hasActiveOutgoingConnection = false;
                for (const auto& conn : scene_.connections()) {
                    // Skip auto-wired connections from generators to the
                    // invisible Output/master module so that only real
                    // module-to-module chains influence centre-line cuts.
                    if (conn.to_module_id == rectai::MASTER_OUTPUT_ID) {
                        continue;
                    }

                    if (conn.from_module_id == object.logical_id()) {
                        const auto toIdIt = std::find_if(
                            objects.begin(), objects.end(),
                            [&conn](const auto& pair) {
                                return pair.second.logical_id() ==
                                       conn.to_module_id;
                            });
                        if (toIdIt != objects.end()) {
                            const auto& toObj = toIdIt->second;
                            if (isInsideMusicArea(toObj) &&
                                (conn.is_hardlink ||
                                 isConnectionGeometricallyActive(
                                     object, toObj))) {
                                hasActiveOutgoingConnection = true;
                                break;
                            }
                        }
                    }
                }

                if (isGeneratorLike && hasActiveOutgoingConnection) {
                    continue;
                }

                const auto cx = bounds.getX() +
                                object.x() * bounds.getWidth();
                const auto cy = bounds.getY() +
                                object.y() * bounds.getHeight();

                // Check intersection with the latest segment.
                const bool intersects = rectai::ui::lineSegmentsIntersect(
                    prevPoint.position, currPoint.position, {cx, cy},
                    centre, detectionThreshold);

                const bool wasIntersecting =
                    touchCurrentlyIntersectingObjects_.find(id) !=
                    touchCurrentlyIntersectingObjects_.end();

                if (intersects && !wasIntersecting) {
                    // Just entered intersection zone: toggle.
                    touchCurrentlyIntersectingObjects_.insert(id);
                    
                    if (touchCutObjects_.find(id) !=
                        touchCutObjects_.end()) {
                        touchCutObjects_.erase(id);
                    } else {
                        touchCutObjects_.insert(id);
                    }
                } else if (!intersects && wasIntersecting) {
                    // Just exited intersection zone: update state.
                    touchCurrentlyIntersectingObjects_.erase(id);
                }
            }

            // Check intersections with module-to-module connections.
            for (const auto& conn : scene_.connections()) {
                const auto fromIt = std::find_if(
                    objects.begin(), objects.end(),
                    [&conn](const auto& pair) {
                        return pair.second.logical_id() ==
                               conn.from_module_id;
                    });
                const auto toIt = std::find_if(
                    objects.begin(), objects.end(),
                    [&conn](const auto& pair) {
                        return pair.second.logical_id() ==
                               conn.to_module_id;
                    });
                if (fromIt == objects.end() || toIt == objects.end()) {
                    continue;
                }

                const auto& fromObj = fromIt->second;
                const auto& toObj = toIt->second;

                if (!isInsideMusicArea(fromObj) ||
                    !isInsideMusicArea(toObj) ||
                    (!conn.is_hardlink &&
                     !isConnectionGeometricallyActive(fromObj, toObj))) {
                    continue;
                }

                const auto fx = bounds.getX() +
                                fromObj.x() * bounds.getWidth();
                const auto fy = bounds.getY() +
                                fromObj.y() * bounds.getHeight();
                const auto tx = bounds.getX() +
                                toObj.x() * bounds.getWidth();
                const auto ty = bounds.getY() +
                                toObj.y() * bounds.getHeight();

                const std::string key = makeConnectionKey(conn);

                // Check intersection with the latest segment.
                const bool intersects = rectai::ui::lineSegmentsIntersect(
                    prevPoint.position, currPoint.position, {fx, fy},
                    {tx, ty}, detectionThreshold);

                const bool wasIntersecting =
                    touchCurrentlyIntersectingConnections_.find(key) !=
                    touchCurrentlyIntersectingConnections_.end();

                if (intersects && !wasIntersecting) {
                    // Just entered intersection zone: toggle.
                    touchCurrentlyIntersectingConnections_.insert(key);
                    
                    if (touchCutConnections_.find(key) !=
                        touchCutConnections_.end()) {
                        touchCutConnections_.erase(key);
                    } else {
                        touchCutConnections_.insert(key);
                    }
                } else if (!intersects && wasIntersecting) {
                    // Just exited intersection zone: update state.
                    touchCurrentlyIntersectingConnections_.erase(key);
                }
            }
        }
    }

    repaint();

    // Dragging the dock scroll area (vertical scroll / pagination).
    if (isDraggingDockScroll_) {
        auto dockBounds = bounds;
        const float dockWidth = calculateDockWidth(dockBounds.getWidth());
        juce::Rectangle<float> dockArea =
            dockBounds.removeFromRight(dockWidth);

        const float titleHeight = 24.0F;
        juce::Rectangle<float> titleArea =
            dockArea.removeFromTop(titleHeight);
        juce::ignoreUnused(titleArea);

        const auto& objects = scene_.objects();
        std::size_t dockCount = 0;
        for (const auto& [id, obj] : objects) {
            if (obj.docked()) {
                ++dockCount;
            }
        }

        if (dockCount == 0) {
            return;
        }

        const float availableHeight = dockArea.getHeight();
        const float nodeRadiusDock = 18.0F;
        const float verticalPadding = 12.0F;
        const float slotHeight = nodeRadiusDock * 2.0F + verticalPadding;
        const float contentHeight =
            slotHeight * static_cast<float>(dockCount);
        const float minOffset =
            (contentHeight > availableHeight)
                ? (availableHeight - contentHeight)
                : 0.0F;

        const float currentY = static_cast<float>(event.position.y);
        const float dy = currentY - dockLastDragY_;
        dockLastDragY_ = currentY;

        dockScrollOffset_ = juce::jlimit(minOffset, 0.0F,
                                         dockScrollOffset_ + dy);
        repaint();
        return;
    }

    // Dragging per-instrument side controls (Freq / Gain).
    if (sideControlKind_ != SideControlKind::kNone &&
        sideControlObjectId_ != 0) {
        auto objects = scene_.objects();
        const auto itObj = objects.find(sideControlObjectId_);
        if (itObj == objects.end()) {
            return;
        }

        const auto& object = itObj->second;
        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        const bool insideMusic = isInsideMusicArea(object);
        float rotationAngle = 0.0F;
        if (insideMusic) {
            const float dx = cx - bounds.getCentreX();
            const float dy = cy - bounds.getCentreY();
            rotationAngle =
                std::atan2(dy, dx) -
                juce::MathConstants<float>::halfPi;
        }

        juce::Point<float> mousePos = event.position;
        if (insideMusic) {
            mousePos = mousePos.transformedBy(
                juce::AffineTransform::rotation(-rotationAngle, cx, cy));
        }

        const float nodeRadius = 26.0F;
        const float ringRadius = nodeRadius + 10.0F;
        const float sliderMargin = 6.0F;
        const float sliderTop = cy - ringRadius + sliderMargin;
        const float sliderBottom = cy + ringRadius - sliderMargin;

        const float mouseY = mousePos.y;
        const float clampedY = juce::jlimit(sliderTop, sliderBottom, mouseY);
        const float value = juce::jmap(clampedY, sliderBottom, sliderTop,
                                       0.0F, 1.0F);

        if (sideControlKind_ == SideControlKind::kFreq) {
            const auto& modulesForFreq = scene_.modules();
            const auto modItFreq =
                modulesForFreq.find(object.logical_id());
            if (modItFreq != modulesForFreq.end() &&
                modItFreq->second != nullptr &&
                modItFreq->second->is<rectai::TempoModule>()) {
                const double minBpm = 40.0;
                const double maxBpm = 400.0;
                const double newBpm =
                    minBpm + (maxBpm - minBpm) *
                                 static_cast<double>(value);
                bpm_ = juce::jlimit(minBpm, maxBpm, newBpm);

                bpmLastChangeSeconds_ =
                    juce::Time::getMillisecondCounterHiRes() /
                    1000.0;

                scene_.SetModuleParameter(object.logical_id(), "tempo",
                                          static_cast<float>(bpm_));
            } else {
                scene_.SetModuleParameter(object.logical_id(), "freq", value);
            }
        } else if (sideControlKind_ == SideControlKind::kGain) {
            const auto& modules = scene_.modules();
            const auto modIt = modules.find(object.logical_id());
            if (modIt != modules.end() && modIt->second != nullptr) {
                auto* module = modIt->second.get();
                if (module->type() == rectai::ModuleType::kFilter) {
                    scene_.SetModuleParameter(object.logical_id(), "q", value);
                } else if (module->is<rectai::VolumeModule>()) {
                    scene_.SetModuleParameter(object.logical_id(), "volume", value);
                } else if (module->is<rectai::LoopModule>() ||
                           module->is<rectai::SampleplayModule>()) {
                    scene_.SetModuleParameter(object.logical_id(), "amp", value);
                } else {
                    scene_.SetModuleParameter(object.logical_id(), "gain", value);
                }
            } else {
                scene_.SetModuleParameter(object.logical_id(), "gain", value);
            }
        }

        repaint();
        return;
    }

    if (draggedObjectId_ == 0) {
        return;
    }

    const float normX = (static_cast<float>(event.position.x) - bounds.getX()) /
                        bounds.getWidth();
    const float normY = (static_cast<float>(event.position.y) - bounds.getY()) /
                        bounds.getHeight();

    auto objects = scene_.objects();
    const auto it = objects.find(draggedObjectId_);
    if (it == objects.end()) {
        return;
    }

    // Prevent objects from overlapping: treat nodes as solid circles
    // and reject moves that would cause intersections with other
    // objects.
    constexpr float nodeRadius = 26.0F;
    const float centreX = bounds.getX() + normX * bounds.getWidth();
    const float centreY = bounds.getY() + normY * bounds.getHeight();
    const float minDist = 2.0F * nodeRadius;
    const float minDistSq = minDist * minDist;

    for (const auto& [otherId, otherObj] : objects) {
        if (otherId == draggedObjectId_) {
            continue;
        }

        const float ox = bounds.getX() + otherObj.x() * bounds.getWidth();
        const float oy = bounds.getY() + otherObj.y() * bounds.getHeight();
        const float dx = centreX - ox;
        const float dy = centreY - oy;
        const float distSq = dx * dx + dy * dy;
        if (distSq < minDistSq) {
            // Block this movement; keep previous position.
            return;
        }
    }

    auto updated = it->second;
    const bool wasDocked = updated.docked();
    const bool wasInsideMusic = isInsideMusicArea(updated);

    if (updated.docked()) {
        // While the pointer is inside the dock area, keep the module
        // docked so it remains visible in the dock list. Only when the
        // drag leaves the dock area do we convert it into a regular
        // object on the table.
        auto dockBounds = bounds;
        const float dockWidth = calculateDockWidth(dockBounds.getWidth());
        juce::Rectangle<float> dockArea =
            dockBounds.removeFromRight(dockWidth);

        if (dockArea.contains(event.position)) {
            // Still inside the dock: do not change Scene; the icon in the
            // dock is the visual feedback while the user aims the drag.
            return;
        }

        // Pointer has left the dock area: instantiate the object on the
        // table at the dragged position, marking it as undocked.
        updated = rectai::ObjectInstance(
            updated.tracking_id(), updated.logical_id(), normX, normY,
            updated.angle_radians(), false);
    } else {
        updated.set_position(normX, normY);
    }

    // Recompute and cache the inside-music-area flag for the
    // updated object at its new position before writing it back
    // into the Scene.
    const bool isNowInsideMusic = computeInsideMusicArea(updated);
    updated.set_inside_music_area(isNowInsideMusic);
    scene_.UpsertObject(updated);

    // When a module becomes active on the musical surface (either by
    // leaving the dock or by entering the circle from outside), normalise
    // its frequency control parameter by explicitly writing its
    // current value back into the Scene. This mirrors the effect of
    // clicking on the freq side control once, ensuring that modules
    // like Filter have their cutoff parameter initialised in the
    // same way even before the user interacts with the slider.
    if ((wasDocked || !wasInsideMusic) && isNowInsideMusic) {
        const auto& modulesForObject = scene_.modules();
        const auto modIt =
            modulesForObject.find(updated.logical_id());
        if (modIt != modulesForObject.end() &&
            modIt->second != nullptr) {
            auto* module = modIt->second.get();

            if (module->uses_frequency_control()) {
                const float currentFreq = module->GetParameterOrDefault(
                    "freq",
                    module->default_parameter_value("freq"));
                scene_.SetModuleParameter(module->id(), "freq",
                                          currentFreq);
            }

            // When a Sampleplay module enters the musical area, also
            // (re)start the visibility timer for its instrument label
            // so that the text is fully visible for a few seconds and
            // then fades out.
            if (auto* sampleModule =
                    dynamic_cast<rectai::SampleplayModule*>(module)) {
                markSampleplayInstrumentLabelActive(
                    sampleModule->id());
            }
        }
    }

    // If CONTROL is held while dragging a module inside the musical
    // area, apply a safety mute by setting its volume to 0 as soon as
    // it is on the table.
    applyControlDropMuteIfNeeded(event);

    repaint();
}

void MainComponent::mouseUp(const juce::MouseEvent& event)
{
    juce::ignoreUnused(event);

    // Handle click-and-hold mute release.
    // Always clear hold state when releasing; connection-level mute
    // changes (if any) are handled via cut gestures.
    if (activeConnectionHold_.has_value()) {
        activeConnectionHold_.reset();
    }
    
    // Apply mute toggle to lines that were cut during touch drag.
    const auto& objects = scene_.objects();
    const auto& connections = scene_.connections();

    // Precompute mapping from module id to object id.
    std::unordered_map<std::string, std::int64_t> moduleToObjectId;
    moduleToObjectId.reserve(objects.size());
    for (const auto& [objId, obj] : objects) {
        moduleToObjectId.emplace(obj.logical_id(), objId);
    }

    // Object-to-centre lines: cutting this line is equivalent to
    // toggling the implicit connection from the module to the master
    // Output (-1). No per-object or per-module mute state is kept; all
    // mute is expressed as connection-level state.
    for (const auto& objectId : touchCutObjects_) {
        const auto objIt = objects.find(objectId);
        if (objIt == objects.end()) {
            continue;
        }

        const auto& obj = objIt->second;
        const std::string moduleId = obj.logical_id();

        bool justUnmutedMaster = false;

        // Find the implicit connection module -> Output (-1) and
        // toggle its mute state.
        for (const auto& conn : connections) {
            if (conn.from_module_id != moduleId ||
                conn.to_module_id != rectai::MASTER_OUTPUT_ID) {
                continue;
            }

            const std::string key = makeConnectionKey(conn);
            const auto it = mutedConnections_.find(key);
            if (it != mutedConnections_.end()) {
                // Master route was muted; unmute it.
                mutedConnections_.erase(it);
                justUnmutedMaster = true;
            } else {
                // Master route was unmuted; mute it.
                mutedConnections_.insert(key);
            }
        }

        // If we have just unmuted the master route for this module,
        // clear any stored per-connection mute state for all edges
        // originating from it, incluyendo conexiones que ya no estén
        // presentes en la Scene pero cuyo estado mute persista en
        // mutedConnections_. Esto modela un "reset global de mute"
        // para el generador: una vez que el usuario des-silencia la
        // radial, todas las rutas salientes (existentes o futuras)
        // parten sin mute heredado.
        if (justUnmutedMaster) {
            const std::string prefix = moduleId + ":";
            std::vector<std::string> keysToErase;
            keysToErase.reserve(mutedConnections_.size());

            for (const auto& key : mutedConnections_) {
                if (key.rfind(prefix, 0) == 0U) {
                    keysToErase.push_back(key);
                }
            }

            for (const auto& key : keysToErase) {
                mutedConnections_.erase(key);
            }
        }
    }

    // For module-to-module connections, cutting a line toggles mute for
    // that specific connection key. Additionally, when this is the only
    // active outgoing connection from the source module (ignoring the
    // implicit module→master auto-wire), we mirror the mute state onto
    // the module→Output(MASTER_OUTPUT_ID) connection so that the mute
    // persists when
    // the topology changes between module→module and module→master.
    for (const auto& connKey : touchCutConnections_) {
        const bool wasMuted =
            mutedConnections_.find(connKey) != mutedConnections_.end();

        const rectai::Connection* matchedConn = nullptr;
        for (const auto& conn : connections) {
            if (makeConnectionKey(conn) == connKey) {
                matchedConn = &conn;
                break;
            }
        }
        std::string srcModuleId;
        bool hasSingleActiveConnection = false;

        if (matchedConn != nullptr) {
            srcModuleId = matchedConn->from_module_id;

            const auto fromObjIt =
                moduleToObjectId.find(srcModuleId);
            const rectai::ObjectInstance* fromObj = nullptr;
            if (fromObjIt != moduleToObjectId.end()) {
                const auto objIt = objects.find(fromObjIt->second);
                if (objIt != objects.end()) {
                    fromObj = &objIt->second;
                }
            }

            if (fromObj != nullptr && isInsideMusicArea(*fromObj)) {
                std::size_t activeOutgoingCount = 0;
                for (const auto& conn : connections) {
                    if (conn.from_module_id != srcModuleId) {
                        continue;
                    }

                    // Ignore auto-wired connections to the invisible
                    // Output/master module so they do not influence the
                    // "only one connection" rule from the user's
                    // perspective.
                    if (conn.to_module_id == rectai::MASTER_OUTPUT_ID) {
                        continue;
                    }

                    const auto toObjIt =
                        moduleToObjectId.find(conn.to_module_id);
                    if (toObjIt == moduleToObjectId.end()) {
                        continue;
                    }

                    const auto objIt = objects.find(toObjIt->second);
                    if (objIt == objects.end()) {
                        continue;
                    }

                    const auto& toObj = objIt->second;
                    if (!isInsideMusicArea(toObj)) {
                        continue;
                    }

                    if (!conn.is_hardlink &&
                        !isConnectionGeometricallyActive(*fromObj,
                                                         toObj)) {
                        continue;
                    }

                    ++activeOutgoingCount;
                }

                hasSingleActiveConnection =
                    (activeOutgoingCount == 1U);
            }
        }

        // Toggle per-connection mute state.
        if (wasMuted) {
            mutedConnections_.erase(connKey);
        } else {
            mutedConnections_.insert(connKey);
        }

        // When muting (not unmuting) the only active outgoing
        // connection from a module, mirror that mute state onto its
        // implicit module→Output(MASTER_OUTPUT_ID) connection so that
        // the module
        // stays muted even if the downstream module is later
        // disconnected and the path reverts to module→master.
        if (matchedConn != nullptr && hasSingleActiveConnection) {
            for (const auto& conn : connections) {
                if (conn.from_module_id != srcModuleId ||
                    conn.to_module_id != rectai::MASTER_OUTPUT_ID) {
                    continue;
                }

                const std::string masterKey = makeConnectionKey(conn);
                if (!wasMuted) {
                    // We are muting the only active connection:
                    // also mute the master route.
                    mutedConnections_.insert(masterKey);
                } else {
                    // We are unmuting that connection: unmute the
                    // master route as well.
                    mutedConnections_.erase(masterKey);
                }
            }
        }
    }

    // Clear touch state.
    isTouchActive_ = false;
    isTouchHeld_ = false;
    touchStartedInDock_ = false;
    touchTrail_.clear();
    touchCutConnections_.clear();
    touchCutObjects_.clear();
    touchCurrentlyIntersectingConnections_.clear();
    touchCurrentlyIntersectingObjects_.clear();

    // Leaving any gesture ends cut mode; the next cut gesture must
    // start explicitly on empty musical space.
    isCutModeActive_ = false;

    repaint();

    draggedObjectId_ = 0;
    sideControlObjectId_ = 0;
    sideControlKind_ = SideControlKind::kNone;
    isDraggingDockScroll_ = false;
}

void MainComponent::applyControlDropMuteIfNeeded(
    const juce::MouseEvent& event)
{
    if (!event.mods.isCtrlDown()) {
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
    if (module->is<rectai::OscillatorModule>()) {
        scene_.SetModuleParameter(obj.logical_id(), "gain", 0.0F);
    } else if (module->is<rectai::LoopModule>() ||
               module->is<rectai::SampleplayModule>()) {
        scene_.SetModuleParameter(obj.logical_id(), "amp", 0.0F);
    }
}
