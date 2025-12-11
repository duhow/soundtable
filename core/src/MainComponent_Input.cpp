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
    // the icon accordingly.
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
        }
        // Right-click that does not hit any oscillator or filter tangible
        // falls through without side effects.
        return;
    }

    // First, try to grab one of the per-instrument side controls
    // (left: Freq, right: Gain) by clicking near its handle.
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

        auto isNearHandle = [](float hx, float hy, juce::Point<float> p) {
            const float dx = p.x - hx;
            const float dy = p.y - hy;
            return (dx * dx + dy * dy) <= 14.0F * 14.0F;
        };

        if (freqEnabled && isNearHandle(freqHandleX, freqHandleY, click)) {
            sideControlObjectId_ = id;
            sideControlKind_ = SideControlKind::kFreq;
            return;
        }
        if (gainEnabled && isNearHandle(gainHandleX, gainHandleY, click)) {
            sideControlObjectId_ = id;
            sideControlKind_ = SideControlKind::kGain;
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
                const auto itMuted = mutedConnections_.find(key);
                if (itMuted == mutedConnections_.end()) {
                    mutedConnections_.insert(key);
                } else {
                    mutedConnections_.erase(itMuted);
                }

                repaint();
                return;
            }
        }

        // Fallback: muting via the line centre  object. Only objects that
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
                const auto itMuted = mutedObjects_.find(id);
                const bool nowMuted = itMuted == mutedObjects_.end();
                if (nowMuted) {
                    mutedObjects_.insert(id);
                } else {
                    mutedObjects_.erase(itMuted);
                }

                repaint();
                break;
            }
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

    repaint();
}

void MainComponent::mouseUp(const juce::MouseEvent&)
{
    // Clear touch state.
    isTouchActive_ = false;
    isTouchHeld_ = false;
    touchStartedInDock_ = false;
    touchTrail_.clear();

    repaint();

    draggedObjectId_ = 0;
    sideControlObjectId_ = 0;
    sideControlKind_ = SideControlKind::kNone;
    isDraggingDockScroll_ = false;
}
