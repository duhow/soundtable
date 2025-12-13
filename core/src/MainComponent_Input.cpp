#include "MainComponent.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "MainComponentHelpers.h"
#include "core/AudioModules.h"

using rectai::ui::isConnectionGeometricallyActive;
using rectai::ui::makeConnectionKey;

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
    const float dockWidth = calculateDockWidth(bounds.getWidth());
    auto boundsCopy = bounds;
    juce::Rectangle<float> dockArea = boundsCopy.removeFromRight(dockWidth);
    touchStartedInDock_ = dockArea.contains(event.position);

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
            if (object.logical_id() == "-1" || object.docked()) {
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
        if (object.logical_id() == "-1") {
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
        bool freqEnabled = false;
        bool gainEnabled = false;
        float freqValue = 0.5F;
        float gainValue = 0.5F;
        if (moduleForObject != nullptr) {
            freqEnabled = moduleForObject->uses_frequency_control();
            gainEnabled = moduleForObject->uses_gain_control();
            freqValue = moduleForObject->GetParameterOrDefault(
                "freq",
                moduleForObject->default_parameter_value("freq"));

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

        auto isOnControlBar = [sliderTop, sliderBottom](float hx,
                                                        juce::Point<float> p) {
            // Treat the side control as a vertical bar around the
            // handle's x position, spanning the full slider height.
            const float halfWidth = 14.0F;
            const float dx = std::abs(p.x - hx);
            const bool withinX = dx <= halfWidth;
            const bool withinY = (p.y >= sliderTop && p.y <= sliderBottom);
            return withinX && withinY;
        };

        // Clicking anywhere on the frequency bar.
        if (freqEnabled && isOnControlBar(freqHandleX, click)) {
            sideControlObjectId_ = id;
            sideControlKind_ = SideControlKind::kFreq;

            const float clampedY = juce::jlimit(sliderTop, sliderBottom, click.y);
            const float value = juce::jmap(clampedY, sliderBottom, sliderTop,
                                           0.0F, 1.0F);

            scene_.SetModuleParameter(object.logical_id(), "freq", value);

            repaint();
            return;
        }

        // Clicking anywhere on the gain bar.
        if (gainEnabled && isOnControlBar(gainHandleX, click)) {
            sideControlObjectId_ = id;
            sideControlKind_ = SideControlKind::kGain;

            const float clampedY = juce::jlimit(sliderTop, sliderBottom, click.y);
            const float value = juce::jmap(clampedY, sliderBottom, sliderTop,
                                           0.0F, 1.0F);

            const auto& modulesForGain = scene_.modules();
            const auto modItGain = modulesForGain.find(object.logical_id());
            if (modItGain != modulesForGain.end() && modItGain->second != nullptr) {
                auto* module = modItGain->second.get();
                if (module->type() == rectai::ModuleType::kFilter) {
                    scene_.SetModuleParameter(object.logical_id(), "q", value);
                } else if (dynamic_cast<rectai::VolumeModule*>(module) != nullptr) {
                    scene_.SetModuleParameter(object.logical_id(), "volume", value);
                } else {
                    scene_.SetModuleParameter(object.logical_id(), "gain", value);
                }
            } else {
                scene_.SetModuleParameter(object.logical_id(), "gain", value);
            }

            repaint();
            return;
        }
    }

    // Next, try to select an object by clicking on its circle.
    for (const auto& [id, object] : objects) {
        if (object.logical_id() == "-1" || object.docked()) {
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
            const float dockWidth = calculateDockWidth(dockBounds.getWidth());
            juce::Rectangle<float> dockArea =
                dockBounds.removeFromRight(dockWidth);

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
                slotHeight * static_cast<float>(dockedObjects.size());
            const float minOffset =
                (contentHeight > availableHeight)
                    ? (availableHeight - contentHeight)
                    : 0.0F;
            dockScrollOffset_ = juce::jlimit(minOffset, 0.0F,
                                             dockScrollOffset_);
            const float baseY = dockArea.getY() + dockScrollOffset_;

            for (std::size_t i = 0; i < dockedObjects.size(); ++i) {
                const auto id = dockedObjects[i].first;

                const float cy = baseY + (static_cast<float>(i) + 0.5F) *
                                            slotHeight;
                const float cx = dockArea.getX() +
                                 dockArea.getWidth() * 0.5F;

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
        const float dockWidth = calculateDockWidth(dockBounds.getWidth());
        juce::Rectangle<float> dockArea =
            dockBounds.removeFromRight(dockWidth);
        if (dockArea.contains(event.position)) {
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
                float splitPoint = 0.5F; // Default to mid-point.
                if (abLenSq > 0.0F) {
                    splitPoint = (ap.x * ab.x + ap.y * ab.y) / abLenSq;
                    splitPoint = juce::jlimit(0.0F, 1.0F, splitPoint);
                }
                
                // Store hold state and apply temporary mute.
                activeConnectionHold_ = ConnectionHoldState{
                    key,
                    0,  // Not an object-to-center line.
                    false,
                    splitPoint
                };
                
                // Always mute while holding.
                mutedConnections_.insert(key);

                repaint();
                return;
            }
        }

        // Fallback: muting via the line centre -> object. Only objects that
        // actually render a visible line to the master in paint() should be
        // clickable here.
        for (const auto& [id, object] : objectsLocal) {
            if (object.logical_id() == "-1" || object.docked()) {
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
            const bool isGenerator =
                modForConnectionIt != modulesLocal.end() &&
                modForConnectionIt->second != nullptr &&
                modForConnectionIt->second->type() ==
                    rectai::ModuleType::kGenerator;

            // Generators feeding another module through an active connection
            // hide their direct visual link to the master, so that line
            // should not be clickable either.
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
                
                // Always mute while holding.
                mutedObjects_.insert(id);

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

            // Check intersections with object-to-center lines.
            for (const auto& [id, object] : objects) {
                if (object.logical_id() == "-1" || object.docked() ||
                    !isInsideMusicArea(object)) {
                    continue;
                }

                // Skip global controllers as they don't have audio lines.
                const auto modIt = modules.find(object.logical_id());
                const rectai::AudioModule* moduleForLine = nullptr;
                if (modIt != modules.end() && modIt->second != nullptr) {
                    moduleForLine = modIt->second.get();
                }
                if (moduleForLine != nullptr &&
                    moduleForLine->is_global_controller()) {
                    continue;
                }

                // Skip generators that are feeding another module.
                const bool isGenerator =
                    moduleForLine != nullptr &&
                    moduleForLine->type() ==
                        rectai::ModuleType::kGenerator;
                
                bool hasActiveOutgoingConnection = false;
                for (const auto& conn : scene_.connections()) {
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

                if (isGenerator && hasActiveOutgoingConnection) {
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
            scene_.SetModuleParameter(object.logical_id(), "freq", value);
        } else if (sideControlKind_ == SideControlKind::kGain) {
            const auto& modules = scene_.modules();
            const auto modIt = modules.find(object.logical_id());
            if (modIt != modules.end() && modIt->second != nullptr) {
                auto* module = modIt->second.get();
                if (module->type() == rectai::ModuleType::kFilter) {
                    scene_.SetModuleParameter(object.logical_id(), "q",
                                              value);
                } else if (dynamic_cast<rectai::VolumeModule*>(module) !=
                           nullptr) {
                    scene_.SetModuleParameter(object.logical_id(), "volume",
                                              value);
                } else {
                    scene_.SetModuleParameter(object.logical_id(), "gain",
                                              value);
                }
            } else {
                scene_.SetModuleParameter(object.logical_id(), "gain",
                                          value);
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
    scene_.UpsertObject(updated);

    // If CONTROL is held while dragging a module inside the musical
    // area, apply a safety mute by setting its volume to 0 as soon as
    // it is on the table.
    applyControlDropMuteIfNeeded(event);

    repaint();
}

void MainComponent::mouseUp(const juce::MouseEvent& event)
{
    // Handle click-and-hold mute release.
    // Always unmute when releasing, regardless of previous state.
    if (activeConnectionHold_.has_value()) {
        const auto& holdState = activeConnectionHold_.value();
        
        if (holdState.is_object_line) {
            mutedObjects_.erase(holdState.object_id);
        } else {
            mutedConnections_.erase(holdState.connection_key);
        }
        
        activeConnectionHold_.reset();
    }
    
    // Apply mute toggle to lines that were cut during touch drag.
    for (const auto& objectId : touchCutObjects_) {
        if (mutedObjects_.find(objectId) != mutedObjects_.end()) {
            mutedObjects_.erase(objectId);
        } else {
            mutedObjects_.insert(objectId);
        }
    }

    for (const auto& connKey : touchCutConnections_) {
        if (mutedConnections_.find(connKey) != mutedConnections_.end()) {
            mutedConnections_.erase(connKey);
        } else {
            mutedConnections_.insert(connKey);
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
    if (dynamic_cast<rectai::OscillatorModule*>(module) != nullptr) {
        scene_.SetModuleParameter(obj.logical_id(), "gain", 0.0F);
    } else if (dynamic_cast<rectai::LoopModule*>(module) != nullptr ||
               dynamic_cast<rectai::SampleplayModule*>(module) != nullptr) {
        scene_.SetModuleParameter(obj.logical_id(), "amp", 0.0F);
    }
}
