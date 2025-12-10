#include "MainComponent.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "AudioEngine.h"
#include "MainComponentHelpers.h"
#include "core/AudioModules.h"

using rectai::ui::colourFromArgb;
using rectai::ui::isConnectionGeometricallyActive;
using rectai::ui::makeConnectionKey;

bool MainComponent::isInsideMusicArea(
    const rectai::ObjectInstance& obj) const
{
    if (obj.docked()) {
        return false;
    }

    const auto bounds = getLocalBounds().toFloat();

    const float centreX = bounds.getX() + 0.5F * bounds.getWidth();
    const float centreY = bounds.getY() + 0.5F * bounds.getHeight();
    const float tableRadius =
        0.45F * std::min(bounds.getWidth(), bounds.getHeight());

    const float x = bounds.getX() + obj.x() * bounds.getWidth();
    const float y = bounds.getY() + obj.y() * bounds.getHeight();

    const float dx = x - centreX;
    const float dy = y - centreY;
    return (dx * dx + dy * dy) <= tableRadius * tableRadius;
}

void MainComponent::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colours::black);

    const auto bounds = getLocalBounds().toFloat();
    const auto centre = bounds.getCentre();

    // ---------------------------------------------------------------------
    // Background: circular table with radial blue gradient and vignette.
    // ---------------------------------------------------------------------
    const float tableRadius =
        0.45F * std::min(bounds.getWidth(), bounds.getHeight());

    juce::Colour centreColour = juce::Colour::fromRGB(0x00, 0x40, 0x80);
    juce::Colour edgeColour = juce::Colour::fromRGB(0x00, 0x10, 0x30);

    juce::ColourGradient gradient(centreColour, centre.x, centre.y, edgeColour,
                                  centre.x, centre.y + tableRadius, true);
    gradient.addColour(0.8, edgeColour.darker(1.2F));

    g.setGradientFill(gradient);
    g.fillEllipse(centre.x - tableRadius, centre.y - tableRadius,
                  tableRadius * 2.0F, tableRadius * 2.0F);

    // Soft vignette around the table.
    g.setColour(juce::Colours::black.withAlpha(0.4F));
    g.drawEllipse(centre.x - tableRadius, centre.y - tableRadius,
                  tableRadius * 2.0F, tableRadius * 2.0F, 20.0F);

    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    std::unordered_map<std::string, std::int64_t> moduleToObjectId;
    for (const auto& [id, object] : objects) {
        moduleToObjectId.emplace(object.logical_id(), id);
    }

    std::unordered_set<std::int64_t> objectsWithOutgoingActiveConnection;
    for (const auto& conn : scene_.connections()) {
        const auto fromIdIt = moduleToObjectId.find(conn.from_module_id);
        const auto toIdIt = moduleToObjectId.find(conn.to_module_id);
        if (fromIdIt == moduleToObjectId.end() ||
            toIdIt == moduleToObjectId.end()) {
            continue;
        }

        const auto fromObjIt = objects.find(fromIdIt->second);
        const auto toObjIt = objects.find(toIdIt->second);
        if (fromObjIt == objects.end() || toObjIt == objects.end()) {
            continue;
        }

        const auto& fromObj = fromObjIt->second;
        const auto& toObj = toObjIt->second;

        if (!isInsideMusicArea(fromObj) || !isInsideMusicArea(toObj)) {
            continue;
        }

        // Hardlink connections are considered logically active regardless
        // of the usual geometric cone constraint between source and
        // destination. Dynamic connections still depend on the cone.
        if (conn.is_hardlink ||
            isConnectionGeometricallyActive(fromObj, toObj)) {
            objectsWithOutgoingActiveConnection.insert(fromIdIt->second);
        }
    }

    {
        juce::Graphics::ScopedSaveState clipState(g);

        // Clip central visuals (background pulses, connections) to the
        // musical area, but allow instruments themselves to be drawn
        // outside.
        juce::Path tableClip;
        tableClip.addEllipse(centre.x - tableRadius, centre.y - tableRadius,
                             tableRadius * 2.0F, tableRadius * 2.0F);
        g.reduceClipRegion(tableClip);

        // -----------------------------------------------------------------
        // Central node: fixed dot + ripples (already BPM-synchronised).
        // Colour and mute state are driven by the Reactable Output
        // tangible (type=Output) when available.
        // -----------------------------------------------------------------
        const float baseRadius = 6.0F;
        const float masterAlpha = masterMuted_ ? 0.4F : 1.0F;
        g.setColour(masterColour_.withAlpha(masterAlpha));
        g.fillEllipse(centre.x - baseRadius, centre.y - baseRadius,
                      baseRadius * 2.0F, baseRadius * 2.0F);

        for (const auto& pulse : pulses_) {
            const float t = juce::jlimit(0.0F, 1.0F, pulse.age);
            const float maxRadius = pulse.strong ? 90.0F : 50.0F;
            const float radius = baseRadius + t * maxRadius;
            const float alpha = (1.0F - t) * (masterMuted_ ? 0.4F : 1.0F);
            const float thickness = pulse.strong ? 5.0F : 2.0F;

            g.setColour(masterColour_.withAlpha(alpha));
            g.drawEllipse(centre.x - radius, centre.y - radius, radius * 2.0F,
                          radius * 2.0F, thickness);
        }

        // -----------------------------------------------------------------
        // Connections: centre each object, with optional flow pulses.
        // All instruments keep their own structural line to the master.
        // When an instrument is feeding another through an active
        // connection, its line remains visible but loses the animated
        // flow pulse so that only the downstream node appears as the
        // active path to the master bus.
        // -----------------------------------------------------------------
        for (const auto& [id, object] : objects) {
            if (object.logical_id() == "-1") {
                continue;  // Output (master) is not drawn as a module.
            }
            const bool isMuted =
                mutedObjects_.find(id) != mutedObjects_.end();

            const bool hasActiveOutgoingConnection =
                objectsWithOutgoingActiveConnection.find(id) !=
                objectsWithOutgoingActiveConnection.end();

            const auto modForConnectionIt =
                modules.find(object.logical_id());
            const bool isGenerator =
                modForConnectionIt != modules.end() &&
                modForConnectionIt->second != nullptr &&
                modForConnectionIt->second->type() ==
                    rectai::ModuleType::kGenerator;

            if (!isInsideMusicArea(object)) {
                continue;
            }

            // When an oscillator is feeding another module through an
            // active connection, it should not render a direct line to
            // the master centre; only the downstream module keeps the
            // visual link to the master bus.
            // Generative modules feeding another one hide their
            // direct visual link to the master; the downstream
            // module becomes the visible path.
            if (isGenerator && hasActiveOutgoingConnection) {
                continue;
            }

            const auto cx = bounds.getX() + object.x() * bounds.getWidth();
            const auto cy = bounds.getY() + object.y() * bounds.getHeight();

            juce::Line<float> line(centre.x, centre.y, cx, cy);

            // Dim the line slightly when this instrument is feeding
            // another one, so the downstream instrument stands out.
            const float lineAlpha = hasActiveOutgoingConnection ? 0.4F : 0.7F;
            g.setColour(juce::Colours::white.withAlpha(lineAlpha));

            if (isMuted) {
                const float dashLengths[] = {6.0F, 4.0F};
                g.drawDashedLine(line, dashLengths,
                                 static_cast<int>(std::size(dashLengths)),
                                 2.0F);
            } else {
                g.drawLine(line, 2.0F);
            }

            // Flow pulse travelling from node to centre to sugerir dirección.
            const float t = static_cast<float>(std::fmod(
                connectionFlowPhase_ +
                    0.25 * static_cast<double>(id % 4),
                1.0));
            const float px = juce::jmap(t, 0.0F, 1.0F, cx, centre.x);
            const float py = juce::jmap(t, 0.0F, 1.0F, cy, centre.y);

            // Only instruments that are not muted and are currently
            // feeding the master directly (i.e. without an active
            // outgoing connection) show the animated pulse.
            const bool showPulse = !isMuted && !hasActiveOutgoingConnection;
            g.setColour(juce::Colours::white.withAlpha(
                showPulse ? 0.9F : 0.0F));
            g.fillEllipse(px - 3.0F, py - 3.0F, 6.0F, 6.0F);
        }

        // -----------------------------------------------------------------
        // Connections between modules (Scene::connections) as curved edges
        // with a small animated pulse suggesting signal flow. Dynamic
        // connections are active only when the target lies inside the
        // 120º cone of the source; hardlink connections remain active
        // regardless of that angular constraint and are drawn in red
        // instead of white.
        // -----------------------------------------------------------------
        int connectionIndex = 0;
        for (const auto& conn : scene_.connections()) {
            const auto fromIt = std::find_if(objects.begin(), objects.end(),
                                             [&conn](const auto& pair) {
                                                 return pair.second.logical_id() ==
                                                        conn.from_module_id;
                                             });
            const auto toIt = std::find_if(objects.begin(), objects.end(),
                                           [&conn](const auto& pair) {
                                               return pair.second.logical_id() ==
                                                      conn.to_module_id;
                                           });
            if (fromIt == objects.end() || toIt == objects.end()) {
                continue;
            }

            const auto& fromObj = fromIt->second;
            const auto& toObj = toIt->second;

            if (!isInsideMusicArea(fromObj) || !isInsideMusicArea(toObj) ||
                (!conn.is_hardlink &&
                 !isConnectionGeometricallyActive(fromObj, toObj))) {
                ++connectionIndex;
                continue;
            }

            const auto fx = bounds.getX() + fromObj.x() * bounds.getWidth();
            const auto fy = bounds.getY() + fromObj.y() * bounds.getHeight();
            const auto tx = bounds.getX() + toObj.x() * bounds.getWidth();
            const auto ty = bounds.getY() + toObj.y() * bounds.getHeight();

            const juce::Point<float> p1{fx, fy};
            const juce::Point<float> p2{tx, ty};
            const juce::Point<float> mid = (p1 + p2) * 0.5F;

            // Control point desplazado ligeramente para dar curvatura.
            const juce::Point<float> delta = p2 - p1;
            const float length =
                std::sqrt(delta.x * delta.x + delta.y * delta.y);
            juce::Point<float> perpDir{0.0F, 0.0F};
            if (length > 0.0F) {
                // Perpendicular vector (rotate 90 degrees CCW)
                perpDir = juce::Point<float>{-delta.y / length * 20.0F,
                                             delta.x / length * 20.0F};
            }
            const juce::Point<float> control = mid + perpDir;
            const bool isMutedConnection =
                mutedConnections_.find(makeConnectionKey(conn)) !=
                mutedConnections_.end();

            const juce::Colour activeColour =
                conn.is_hardlink
                    ? juce::Colours::red.withAlpha(0.8F)
                    : juce::Colours::white.withAlpha(0.7F);

            if (isMutedConnection) {
                // Muted connection: render as a dashed straight line
                // between the two nodes and omit the animated flow
                // pulse.
                const float dashLengths[] = {6.0F, 4.0F};
                g.setColour(activeColour.withAlpha(0.6F));
                g.drawDashedLine(juce::Line<float>(p1, p2), dashLengths,
                                 static_cast<int>(std::size(dashLengths)),
                                 1.5F);
            } else {
                juce::Path path;
                path.startNewSubPath(p1);
                path.quadraticTo(control, p2);

                g.setColour(activeColour);
                g.strokePath(path, juce::PathStrokeType(1.5F));

                // Animated pulse travelling along the curved
                // connection.
                const double phaseOffset =
                    0.25 * static_cast<double>(connectionIndex);
                const float t = static_cast<float>(
                    std::fmod(connectionFlowPhase_ + phaseOffset, 1.0));
                const float oneMinusT = 1.0F - t;

                const juce::Point<float> flowPoint =
                    oneMinusT * oneMinusT * p1 +
                    2.0F * oneMinusT * t * control + t * t * p2;

                g.setColour(conn.is_hardlink
                                 ? juce::Colours::red.withAlpha(0.9F)
                                 : juce::Colours::white.withAlpha(0.85F));
                g.fillEllipse(flowPoint.x - 2.5F, flowPoint.y - 2.5F, 5.0F,
                              5.0F);
            }

            ++connectionIndex;
        }
    }

    // ---------------------------------------------------------------------
    // Objects: aura + parameter arcs + icon + label.
    // ---------------------------------------------------------------------
    const float nodeRadius = 26.0F;

    auto getBodyColourForObject = [&modules](const rectai::ObjectInstance& obj,
                                             bool isMuted) {
        juce::Colour activeBase = juce::Colour::fromRGB(0x20, 0x90, 0xFF);

        const auto it = modules.find(obj.logical_id());
        if (it != modules.end() && it->second != nullptr) {
            activeBase = colourFromArgb(it->second->colour_argb());
        }

        if (isMuted) {
            return activeBase.darker(1.5F).withMultipliedAlpha(0.8F);
        }

        return activeBase;
    };

    for (const auto& entry : objects) {
        const auto& object = entry.second;
        if (object.docked() || object.logical_id() == "-1") {
            continue;
        }
        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        const bool isMuted =
            mutedObjects_.find(entry.first) != mutedObjects_.end();

        const auto bodyColour = getBodyColourForObject(object, isMuted);

        const juce::Colour auraColour =
            bodyColour.withMultipliedAlpha(0.22F).brighter(0.2F);

        // Aura: soft glow under the object.
        const float glowRadiusOuter = nodeRadius + 22.0F;
        const float glowRadiusInner = nodeRadius + 10.0F;

        g.setColour(auraColour.withMultipliedAlpha(0.5F));
        g.fillEllipse(cx - glowRadiusOuter, cy - glowRadiusOuter,
                      glowRadiusOuter * 2.0F, glowRadiusOuter * 2.0F);

        g.setColour(auraColour);
        g.fillEllipse(cx - glowRadiusInner, cy - glowRadiusInner,
                      glowRadiusInner * 2.0F, glowRadiusInner * 2.0F);

        // Node body.
        g.setColour(bodyColour);
        g.fillEllipse(cx - nodeRadius, cy - nodeRadius, nodeRadius * 2.0F,
                      nodeRadius * 2.0F);

        // Side controls: left = curved bar (semi-arc) for Freq,
        // right = curved bar (semi-arc) for Gain, both hugging the
        // left/right side of the node.
        const float ringRadius = nodeRadius + 10.0F;

        const rectai::AudioModule* moduleForObject = nullptr;
        if (const auto moduleEntryIt = modules.find(object.logical_id());
            moduleEntryIt != modules.end()) {
            moduleForObject = moduleEntryIt->second.get();
        }
        float freqValue = 0.5F;
        float gainValue = 0.5F;
        bool showFreqControl = false;
        bool showGainControl = false;
        if (moduleForObject != nullptr) {
            freqValue = moduleForObject->GetParameterOrDefault(
                "freq",
                moduleForObject->default_parameter_value("freq"));
            gainValue = moduleForObject->GetParameterOrDefault(
                "gain",
                moduleForObject->default_parameter_value("gain"));
            showFreqControl = moduleForObject->uses_frequency_control();
            showGainControl = moduleForObject->uses_gain_control();
        }
        freqValue = juce::jlimit(0.0F, 1.0F, freqValue);
        gainValue = juce::jlimit(0.0F, 1.0F, gainValue);

        const float sliderMargin = 6.0F;
        const float sliderTop = cy - ringRadius + sliderMargin;
        const float sliderBottom = cy + ringRadius - sliderMargin;

        // Left control (Freq): curved bar following the left semi-circle.
        if (showFreqControl) {
            juce::Path freqArc;
            const int segments = 40;
            for (int i = 0; i <= segments; ++i) {
                const float t = static_cast<float>(i) /
                                static_cast<float>(segments);
                const float y = juce::jmap(t, 0.0F, 1.0F,
                                           sliderTop, sliderBottom);
                const float dy = y - cy;
                const float inside = ringRadius * ringRadius - dy * dy;
                if (inside < 0.0F) {
                    continue;
                }
                const float dx = std::sqrt(inside);
                const float x = cx - dx;
                if (i == 0) {
                    freqArc.startNewSubPath(x, y);
                } else {
                    freqArc.lineTo(x, y);
                }
            }
            g.setColour(juce::Colours::white.withAlpha(0.7F));
            g.strokePath(freqArc, juce::PathStrokeType(3.0F));

            const float handleY = juce::jmap(freqValue, 0.0F, 1.0F,
                                             sliderBottom, sliderTop);
            const float dy = handleY - cy;
            const float inside = ringRadius * ringRadius - dy * dy;
            if (inside >= 0.0F) {
                const float dx = std::sqrt(inside);
                const float handleX = cx - dx;
                g.setColour(juce::Colours::white);
                g.fillEllipse(handleX - 6.0F, handleY - 6.0F, 12.0F, 12.0F);
            }
        }

        // Right control (Gain): curved bar following the right semi-circle.
        if (showGainControl) {
            juce::Path gainArc;
            const int segments = 40;
            for (int i = 0; i <= segments; ++i) {
                const float t = static_cast<float>(i) /
                                static_cast<float>(segments);
                const float y = juce::jmap(t, 0.0F, 1.0F,
                                           sliderTop, sliderBottom);
                const float dy = y - cy;
                const float inside = ringRadius * ringRadius - dy * dy;
                if (inside < 0.0F) {
                    continue;
                }
                const float dx = std::sqrt(inside);
                const float x = cx + dx;
                if (i == 0) {
                    gainArc.startNewSubPath(x, y);
                } else {
                    gainArc.lineTo(x, y);
                }
            }
            g.setColour(juce::Colours::white.withAlpha(0.5F));
            g.strokePath(gainArc, juce::PathStrokeType(2.0F));

            const float handleY = juce::jmap(gainValue, 0.0F, 1.0F,
                                             sliderBottom, sliderTop);
            const float dy = handleY - cy;
            const float inside = ringRadius * ringRadius - dy * dy;
            if (inside >= 0.0F) {
                const float dx = std::sqrt(inside);
                const float handleX = cx + dx;
                g.setColour(juce::Colours::white);
                g.fillEllipse(handleX - 6.0F, handleY - 6.0F, 12.0F, 12.0F);
            }
        }

        // Icon inside the node body depending on metadata. Prefer the
        // original Reactable sprite atlas when available; fall back to
        // the existing procedural icons otherwise. When a sprite exists,
        // we no longer draw the textual label inside the node, so the
        // icon becomes the primary visual identifier.
        std::string iconId;
        if (moduleForObject != nullptr) {
            iconId = moduleForObject->icon_id();
        }

        const float iconSize = nodeRadius * 1.4F;
        const juce::Rectangle<float> iconBounds(cx - iconSize * 0.5F,
                                                cy - iconSize * 0.5F,
                                                iconSize, iconSize);

        bool drewAtlasIcon = false;
        if (atlasLoaded_ && !iconId.empty()) {
            const auto spriteIt = atlasSprites_.find(iconId);
            if (spriteIt != atlasSprites_.end() && atlasImage_.isValid()) {
                const auto& src = spriteIt->second.bounds;

                const int destX = juce::roundToInt(iconBounds.getX());
                const int destY = juce::roundToInt(iconBounds.getY());
                const int destW = juce::roundToInt(iconBounds.getWidth());
                const int destH = juce::roundToInt(iconBounds.getHeight());

                g.drawImage(atlasImage_, destX, destY, destW, destH,
                            src.getX(), src.getY(), src.getWidth(),
                            src.getHeight());
                drewAtlasIcon = true;
            }
        }

        if (!drewAtlasIcon) {
            const float iconRadius = nodeRadius * 0.6F;
            const float left = cx - iconRadius;
            const float right = cx + iconRadius;
            const float top = cy - iconRadius * 0.5F;
            const float midY = cy;
            const float bottom = cy + iconRadius * 0.5F;

            g.setColour(juce::Colours::white.withAlpha(0.9F));

            if (iconId == "oscillator") {
                juce::Path wave;
                const int segments = 24;
                for (int i = 0; i <= segments; ++i) {
                    const float t = static_cast<float>(i) /
                                    static_cast<float>(segments);
                    const float x = juce::jmap(t, 0.0F, 1.0F, left, right);
                    const float s =
                        std::sin(t * juce::MathConstants<float>::twoPi);
                    const float y = midY - s * iconRadius * 0.4F;
                    if (i == 0) {
                        wave.startNewSubPath(x, y);
                    } else {
                        wave.lineTo(x, y);
                    }
                }
                g.strokePath(wave, juce::PathStrokeType(1.6F));
            } else if (iconId == "filter") {
                juce::Path curve;
                curve.startNewSubPath(left, bottom);
                curve.quadraticTo({cx, top}, {right, midY});
                g.strokePath(curve, juce::PathStrokeType(1.6F));
            } else if (iconId == "sampler") {
                const float barWidth = (right - left) / 6.0F;
                for (int i = 0; i < 6; ++i) {
                    const float x = left + (static_cast<float>(i) + 0.5F) *
                                             barWidth;
                    const float hFactor = 0.3F + 0.1F *
                                                        static_cast<float>(i);
                    const float yTop = midY - iconRadius * hFactor;
                    const float yBottom = midY + iconRadius * 0.3F;
                    g.drawLine(x, yTop, x, yBottom, 1.5F);
                }
            } else if (iconId == "effect") {
                const float r = iconRadius * 0.6F;
                g.drawEllipse(cx - r, cy - r, r * 2.0F, r * 2.0F, 1.5F);
                for (int i = 0; i < 3; ++i) {
                    const float a = juce::MathConstants<float>::twoPi *
                                    static_cast<float>(i) / 3.0F;
                    const float ex = cx + r * std::cos(a);
                    const float ey = cy + r * std::sin(a);
                    g.fillEllipse(ex - 2.0F, ey - 2.0F, 4.0F, 4.0F);
                }
            } else if (iconId == "controller") {
                const float innerR = iconRadius * 0.4F;
                const float outerR = iconRadius * 0.8F;
                g.drawEllipse(cx - innerR, cy - innerR, innerR * 2.0F,
                              innerR * 2.0F, 1.2F);
                g.drawEllipse(cx - outerR, cy - outerR, outerR * 2.0F,
                              outerR * 2.0F, 1.2F);
            } else {
                // As a last resort, keep the textual label centred in the
                // node so that modules without an icon remain legible.
                juce::String fallbackLabel(object.logical_id());
                if (moduleForObject != nullptr) {
                    const auto& moduleLabel = moduleForObject->label();
                    if (!moduleLabel.empty()) {
                        fallbackLabel = moduleLabel + " (" +
                                        object.logical_id() + ")";
                    }
                }
                g.setColour(juce::Colours::white);
                g.setFont(14.0F);
                g.drawText(fallbackLabel,
                           juce::Rectangle<float>(cx - nodeRadius,
                                                  cy - nodeRadius,
                                                  nodeRadius * 2.0F,
                                                  nodeRadius * 2.0F),
                           juce::Justification::centred, false);
            }
        }
    }

    // ---------------------------------------------------------------------
    // Sequencer widget overlays.
    // ---------------------------------------------------------------------
    for (const auto& entry : objects) {
        const auto& object = entry.second;
        if (object.docked() || object.logical_id() == "-1") {
            continue;
        }
        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        // Simple sequencer: attach to any object whose logical_id starts
        // with "seq" for now. The grid is positioned using the
        // tangibles rotation so that it "projects" outwards.
        if (object.logical_id().rfind("seq", 0) == 0) {
            const int cols = 8;
            const int rows = 3;
            const float cellSize = 14.0F;
            const float gridWidth = cols * cellSize;
            const float gridHeight = rows * cellSize;

            const float angle = object.angle_radians();
            const float offset = nodeRadius + 40.0F;
            const float gx = cx + offset * std::cos(angle);
            const float gy = cy + offset * std::sin(angle);

            juce::Rectangle<float> gridBounds(gx - gridWidth * 0.5F,
                                              gy - gridHeight * 0.5F,
                                              gridWidth, gridHeight);

            g.setColour(juce::Colours::white.withAlpha(0.2F));
            g.drawRoundedRectangle(gridBounds, 4.0F, 1.0F);

            // Draw grid points.
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {
                    const float x = gridBounds.getX() + (c + 0.5F) * cellSize;
                    const float y = gridBounds.getY() + (r + 0.5F) * cellSize;

                    const bool isActiveStep = (c == sequencerStep_);
                    const float alpha = isActiveStep ? 0.9F : 0.3F;
                    const float radius = isActiveStep ? 3.5F : 2.0F;

                    g.setColour(juce::Colours::white.withAlpha(alpha));
                    g.fillEllipse(x - radius, y - radius, radius * 2.0F,
                                  radius * 2.0F);
                }
            }
        }
    }

    // ---------------------------------------------------------------------
    // Dock bar: stacked modules that are marked as docked in the .rtp.
    // These modules live outside the main musical surface but remain
    // visible and accessible in a dedicated side strip.
    // ---------------------------------------------------------------------
    std::vector<std::pair<std::int64_t, const rectai::ObjectInstance*>>
        dockedObjects;
    dockedObjects.reserve(objects.size());
    for (const auto& [id, obj] : objects) {
        if (obj.docked()) {
            dockedObjects.emplace_back(id, &obj);
        }
    }

    if (!dockedObjects.empty()) {
        auto dockBounds = bounds;
        const float dockWidth =
            juce::jmin(220.0F, dockBounds.getWidth() * 0.28F);
        juce::Rectangle<float> dockArea =
            dockBounds.removeFromRight(dockWidth);

        // Background panel.
        g.setColour(juce::Colours::black.withAlpha(0.75F));
        g.fillRoundedRectangle(dockArea, 10.0F);
        g.setColour(juce::Colours::white.withAlpha(0.25F));
        g.drawRoundedRectangle(dockArea, 10.0F, 1.5F);

        // Title.
        g.setColour(juce::Colours::white.withAlpha(0.7F));
        g.setFont(15.0F);
        const float titleHeight = 24.0F;
        juce::Rectangle<float> titleArea =
            dockArea.removeFromTop(titleHeight);
        g.drawText("Dock", titleArea.reduced(6.0F, 0.0F),
                   juce::Justification::centredLeft, false);

        if (!dockedObjects.empty()) {
            std::sort(dockedObjects.begin(), dockedObjects.end(),
                      [](const auto& a, const auto& b) {
                          return a.first < b.first;
                      });

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
                const auto* obj = dockedObjects[i].second;

                const float cy = baseY +
                                 (static_cast<float>(i) + 0.5F) *
                                     slotHeight;
                const float cx = dockArea.getX() +
                                 dockArea.getWidth() * 0.5F;

                // Skip items that are far outside the visible dock area.
                if (cy + nodeRadiusDock < dockArea.getY() ||
                    cy - nodeRadiusDock > dockArea.getBottom()) {
                    continue;
                }

                const bool isMuted =
                    mutedObjects_.find(id) != mutedObjects_.end();
                const auto bodyColour =
                    getBodyColourForObject(*obj, isMuted);
                const auto auraColour =
                    bodyColour.withMultipliedAlpha(0.22F).brighter(0.15F);

                const float glowRadius = nodeRadiusDock + 14.0F;
                g.setColour(auraColour.withMultipliedAlpha(0.6F));
                g.fillEllipse(cx - glowRadius, cy - glowRadius,
                              glowRadius * 2.0F, glowRadius * 2.0F);

                g.setColour(bodyColour);
                g.fillEllipse(cx - nodeRadiusDock, cy - nodeRadiusDock,
                              nodeRadiusDock * 2.0F,
                              nodeRadiusDock * 2.0F);

                const rectai::AudioModule* moduleForObject = nullptr;
                if (const auto moduleEntryIt = modules.find(
                        obj->logical_id());
                    moduleEntryIt != modules.end()) {
                    moduleForObject = moduleEntryIt->second.get();
                }

                std::string iconId;
                if (moduleForObject != nullptr) {
                    iconId = moduleForObject->icon_id();
                }

                const float iconSize = nodeRadiusDock * 1.4F;
                const juce::Rectangle<float> iconBounds(
                    cx - iconSize * 0.5F, cy - iconSize * 0.5F, iconSize,
                    iconSize);

                bool drewAtlasIcon = false;
                if (atlasLoaded_ && !iconId.empty()) {
                    const auto spriteIt = atlasSprites_.find(iconId);
                    if (spriteIt != atlasSprites_.end() &&
                        atlasImage_.isValid()) {
                        const auto& src = spriteIt->second.bounds;
                        const int destX =
                            juce::roundToInt(iconBounds.getX());
                        const int destY =
                            juce::roundToInt(iconBounds.getY());
                        const int destW =
                            juce::roundToInt(iconBounds.getWidth());
                        const int destH =
                            juce::roundToInt(iconBounds.getHeight());

                        g.drawImage(atlasImage_, destX, destY, destW, destH,
                                    src.getX(), src.getY(), src.getWidth(),
                                    src.getHeight());
                        drewAtlasIcon = true;
                    }
                }

                if (!drewAtlasIcon) {
                    // Fallback: reuse simple procedural icons; no text in
                    // the dock to keep it clean.
                    const float iconRadius = nodeRadiusDock * 0.7F;
                    const float left = cx - iconRadius;
                    const float right = cx + iconRadius;
                    const float top = cy - iconRadius * 0.5F;
                    const float midY = cy;
                    const float bottom = cy + iconRadius * 0.5F;

                    g.setColour(juce::Colours::white.withAlpha(0.9F));

                    if (iconId == "oscillator") {
                        juce::Path wave;
                        const int segments = 20;
                        for (int i = 0; i <= segments; ++i) {
                            const float t = static_cast<float>(i) /
                                            static_cast<float>(segments);
                            const float x = juce::jmap(
                                t, 0.0F, 1.0F, left, right);
                            const float s = std::sin(
                                t * juce::MathConstants<float>::twoPi);
                            const float y =
                                midY - s * iconRadius * 0.4F;
                            if (i == 0) {
                                wave.startNewSubPath(x, y);
                            } else {
                                wave.lineTo(x, y);
                            }
                        }
                        g.strokePath(wave, juce::PathStrokeType(1.4F));
                    } else if (iconId == "filter") {
                        juce::Path curve;
                        curve.startNewSubPath(left, bottom);
                        curve.quadraticTo({cx, top}, {right, midY});
                        g.strokePath(curve, juce::PathStrokeType(1.4F));
                    } else if (iconId == "effect") {
                        const float r = iconRadius * 0.6F;
                        g.drawEllipse(cx - r, cy - r, r * 2.0F, r * 2.0F,
                                      1.4F);
                    }
                }
            }
        }
    }
}

void MainComponent::resized()
{
}
