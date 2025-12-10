#include "MainComponent.h"

#include <algorithm>
#include <cmath>

#include "AudioEngine.h"

namespace {

bool isConnectionGeometricallyActive(const rectai::ObjectInstance& fromObj,
                                     const rectai::ObjectInstance& toObj)
{
    const float dx = toObj.x() - fromObj.x();
    const float dy = toObj.y() - fromObj.y();
    const float distanceSq = dx * dx + dy * dy;

    // Distance threshold in normalised table coordinates.
    constexpr float kMaxDistanceSq = 0.25F;  // ~0.5 units.
    if (distanceSq > kMaxDistanceSq) {
        return false;
    }

    const float dist = std::sqrt(distanceSq);
    if (dist <= 1.0e-4F) {
        return false;
    }

    const float dirX = dx / dist;
    const float dirY = dy / dist;

    const float angle = fromObj.angle_radians();
    const float facingX = std::cos(angle);
    const float facingY = std::sin(angle);

    // Require that the target is roughly "in front" of the object.
    const float dot = dirX * facingX + dirY * facingY;
    constexpr float kMinDot = 0.3F;  // ~70 degrees field of view.
    return dot > kMinDot;
}

}  // namespace

MainComponent::MainComponent(AudioEngine& audioEngine)
    : audioEngine_(audioEngine)
{
    setSize(1280, 720);

    // Example scene with a couple of modules and objects.
    rectai::Module osc1("osc1", rectai::ModuleKind::kOscillator);
    osc1.AddOutputPort("out", true);

    rectai::Module filter1("filter1", rectai::ModuleKind::kFilter);
    filter1.AddInputPort("in", true);
    filter1.AddOutputPort("out", true);

    (void)scene_.AddModule(osc1);
    (void)scene_.AddModule(filter1);

    rectai::Connection connection{
        .from_module_id = "osc1",
        .from_port_name = "out",
        .to_module_id = "filter1",
        .to_port_name = "in",
    };
    (void)scene_.AddConnection(connection);

    // Normalized positions on the table.
    scene_.UpsertObject(rectai::ObjectInstance(1, "osc1", 0.3F, 0.5F, 0.0F));
    scene_.UpsertObject(
        rectai::ObjectInstance(2, "filter1", 0.7F, 0.5F, 0.0F));

    // Periodically map scene state to audio parameters.
    startTimerHz(60);
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

    // Clip all further drawing to the circular table area.
    juce::Path tableClip;
    tableClip.addEllipse(centre.x - tableRadius, centre.y - tableRadius,
                         tableRadius * 2.0F, tableRadius * 2.0F);
    g.reduceClipRegion(tableClip);

    // ---------------------------------------------------------------------
    // Central node: fixed dot + ripples (already BPM-synchronised).
    // ---------------------------------------------------------------------
    const float baseRadius = 6.0F;
    g.setColour(juce::Colours::white);
    g.fillEllipse(centre.x - baseRadius, centre.y - baseRadius,
                  baseRadius * 2.0F, baseRadius * 2.0F);

    for (const auto& pulse : pulses_) {
        const float t = juce::jlimit(0.0F, 1.0F, pulse.age);
        const float maxRadius = pulse.strong ? 90.0F : 70.0F;
        const float radius = baseRadius + t * maxRadius;
        const float alpha = 1.0F - t;
        const float thickness = pulse.strong ? 3.0F : 2.0F;

        g.setColour(juce::Colours::white.withAlpha(alpha));
        g.drawEllipse(centre.x - radius, centre.y - radius, radius * 2.0F,
                      radius * 2.0F, thickness);
    }

    // ---------------------------------------------------------------------
    // Connections: centre → each object, with optional flow pulses.
    // Instruments that are actively feeding another module do not draw a
    // direct line to the master; the downstream module's line represents
    // the whole chain.
    // ---------------------------------------------------------------------
    const auto& objects = scene_.objects();

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

        if (isConnectionGeometricallyActive(fromObjIt->second,
                                            toObjIt->second)) {
            objectsWithOutgoingActiveConnection.insert(fromIdIt->second);
        }
    }

    g.setColour(juce::Colours::white.withAlpha(0.7F));
    for (const auto& [id, object] : objects) {
        if (objectsWithOutgoingActiveConnection.find(id) !=
            objectsWithOutgoingActiveConnection.end()) {
            // This object is sending audio to another instrument; its
            // chain is represented by the downstream line to master.
            continue;
        }

        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        juce::Line<float> line(centre.x, centre.y, cx, cy);

        const bool isMuted =
            mutedObjects_.find(id) != mutedObjects_.end();

        if (isMuted) {
            const float dashLengths[] = {6.0F, 4.0F};
            g.drawDashedLine(line, dashLengths,
                             static_cast<int>(std::size(dashLengths)), 2.0F);
        } else {
            g.drawLine(line, 2.0F);
        }

        // Flow pulse travelling from node to centre to sugerir dirección.
        const float t = static_cast<float>(std::fmod(connectionFlowPhase_ +
                                                         0.25 * (id % 4),
                                                     1.0));
        const float px = juce::jmap(t, 0.0F, 1.0F, cx, centre.x);
        const float py = juce::jmap(t, 0.0F, 1.0F, cy, centre.y);

        g.setColour(juce::Colours::white.withAlpha(isMuted ? 0.0F : 0.9F));
        g.fillEllipse(px - 3.0F, py - 3.0F, 6.0F, 6.0F);
    }

    // ---------------------------------------------------------------------
    // Connections between modules (Scene::connections) as curved edges
    // with a small animated pulse suggesting signal flow. Only active when
    // the target is close and roughly "in front" of the source.
    // ---------------------------------------------------------------------
    g.setColour(juce::Colours::white.withAlpha(0.4F));
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

        if (!isConnectionGeometricallyActive(fromObj, toObj)) {
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
        // Compute a perpendicular vector of fixed magnitude (20.0F)
        const juce::Point<float> delta = p2 - p1;
        const float length = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        juce::Point<float> perpDir{0.0F, 0.0F};
        if (length > 0.0F) {
            // Perpendicular vector (rotate 90 degrees CCW)
            perpDir = juce::Point<float>{-delta.y / length * 20.0F, delta.x / length * 20.0F};
        }
        const juce::Point<float> control = mid + perpDir;

        juce::Path path;
        path.startNewSubPath(p1);
        path.quadraticTo(control, p2);

        g.strokePath(path, juce::PathStrokeType(1.5F));

        // Animated pulse travelling along the curved connection.
        const double phaseOffset = 0.25 * static_cast<double>(connectionIndex);
        const float t = static_cast<float>(std::fmod(
            connectionFlowPhase_ + phaseOffset, 1.0));
        const float oneMinusT = 1.0F - t;

        const juce::Point<float> flowPoint = oneMinusT * oneMinusT * p1 +
                                             2.0F * oneMinusT * t * control +
                                             t * t * p2;

        g.setColour(juce::Colours::white.withAlpha(0.85F));
        g.fillEllipse(flowPoint.x - 2.5F, flowPoint.y - 2.5F, 5.0F, 5.0F);

        ++connectionIndex;
    }

    // ---------------------------------------------------------------------
    // Objects: aura + parameter arcs + icon + label.
    // ---------------------------------------------------------------------
    const float nodeRadius = 26.0F;

    const auto& modules = scene_.modules();

    auto getBodyColourForObject = [&modules](const rectai::ObjectInstance& obj,
                                             bool isMuted) {
        juce::Colour activeBase = juce::Colour::fromRGB(0x20, 0x90, 0xFF);

        const auto it = modules.find(obj.logical_id());
        if (it != modules.end()) {
            switch (it->second.kind()) {
            case rectai::ModuleKind::kOscillator:
                activeBase = juce::Colour::fromRGB(0x20, 0x90, 0xFF);
                break;
            case rectai::ModuleKind::kFilter:
                activeBase = juce::Colour::fromRGB(0x40, 0xE0, 0xA0);
                break;
            case rectai::ModuleKind::kEffect:
                activeBase = juce::Colour::fromRGB(0xC0, 0x60, 0xFF);
                break;
            case rectai::ModuleKind::kSampler:
                activeBase = juce::Colour::fromRGB(0xFF, 0xA0, 0x40);
                break;
            case rectai::ModuleKind::kController:
                activeBase = juce::Colour::fromRGB(0x60, 0xF0, 0xFF);
                break;
            default:
                break;
            }
        }

        if (isMuted) {
            return activeBase.darker(1.5F).withMultipliedAlpha(0.8F);
        }

        return activeBase;
    };

    for (const auto& entry : objects) {
        const auto& object = entry.second;
        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        const bool isMuted =
            mutedObjects_.find(entry.first) != mutedObjects_.end();

        const auto bodyColour = getBodyColourForObject(object, isMuted);

        const juce::Colour auraColour =
            bodyColour.withMultipliedAlpha(0.22F).brighter(0.2F);

        const float objectAngle = object.angle_radians();

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

        // Parameter arcs: inner ring for a value based on x position,
        // outer ring for a cycle / progress based on the global beat.
        const float paramRadiusInner = nodeRadius + 8.0F;
        const float paramRadiusOuter = nodeRadius + 14.0F;

        const float normX = juce::jlimit(0.0F, 1.0F, object.x());
        const float innerSpan = normX * juce::MathConstants<float>::twoPi;

        const float beat = static_cast<float>(beatPhase_);
        const float outerSpan = beat * juce::MathConstants<float>::twoPi;

        const float baseAngle = objectAngle -
                                juce::MathConstants<float>::halfPi;

        const float innerStart = baseAngle;
        const float innerEnd = innerStart + innerSpan;

        if (innerSpan > 0.01F) {
            juce::Path innerArc;
            innerArc.addCentredArc(cx, cy, paramRadiusInner, paramRadiusInner,
                                   0.0F, innerStart, innerEnd, true);

            g.setColour(juce::Colours::white.withAlpha(0.9F));
            g.strokePath(innerArc, juce::PathStrokeType(2.0F));

            const float markerX =
                cx + paramRadiusInner * std::cos(innerEnd);
            const float markerY =
                cy + paramRadiusInner * std::sin(innerEnd);
            g.fillEllipse(markerX - 3.0F, markerY - 3.0F, 6.0F, 6.0F);
        }

        if (outerSpan > 0.02F) {
            juce::Path outerArc;
            outerArc.addCentredArc(cx, cy, paramRadiusOuter, paramRadiusOuter,
                                   0.0F, baseAngle, baseAngle + outerSpan,
                                   true);

            g.setColour(juce::Colours::white.withAlpha(0.55F));
            g.strokePath(outerArc, juce::PathStrokeType(2.0F));
        }

        // Waveform / icon inside the node body depending on ModuleKind.
        const auto moduleIt = modules.find(object.logical_id());
        if (moduleIt != modules.end()) {
            const auto kind = moduleIt->second.kind();

            const float iconRadius = nodeRadius * 0.6F;
            const float left = cx - iconRadius;
            const float right = cx + iconRadius;
            const float top = cy - iconRadius * 0.5F;
            const float midY = cy;
            const float bottom = cy + iconRadius * 0.5F;

            g.setColour(juce::Colours::white.withAlpha(0.9F));

            switch (kind) {
            case rectai::ModuleKind::kOscillator: {
                juce::Path wave;
                const int segments = 24;
                for (int i = 0; i <= segments; ++i) {
                    const float t = static_cast<float>(i) /
                                    static_cast<float>(segments);
                    const float x = juce::jmap(t, 0.0F, 1.0F, left, right);
                    const float s = std::sin(t * juce::MathConstants<float>::twoPi);
                    const float y = midY - s * iconRadius * 0.4F;
                    if (i == 0) {
                        wave.startNewSubPath(x, y);
                    } else {
                        wave.lineTo(x, y);
                    }
                }
                g.strokePath(wave, juce::PathStrokeType(1.6F));
                break;
            }
            case rectai::ModuleKind::kFilter: {
                juce::Path curve;
                curve.startNewSubPath(left, bottom);
                curve.quadraticTo({cx, top}, {right, midY});
                g.strokePath(curve, juce::PathStrokeType(1.6F));
                break;
            }
            case rectai::ModuleKind::kSampler: {
                const float barWidth = (right - left) / 6.0F;
                for (int i = 0; i < 6; ++i) {
                    const float x = left + (static_cast<float>(i) + 0.5F) *
                                             barWidth;
                    const float hFactor = 0.3F + 0.1F * static_cast<float>(i);
                    const float yTop =
                        midY - iconRadius * hFactor;
                    const float yBottom = midY + iconRadius * 0.3F;
                    g.drawLine(x, yTop, x, yBottom, 1.5F);
                }
                break;
            }
            case rectai::ModuleKind::kEffect: {
                const float r = iconRadius * 0.6F;
                g.drawEllipse(cx - r, cy - r, r * 2.0F, r * 2.0F, 1.5F);
                for (int i = 0; i < 3; ++i) {
                    const float a = objectAngle +
                                    juce::MathConstants<float>::twoPi *
                                        static_cast<float>(i) / 3.0F;
                    const float ex = cx + r * std::cos(a);
                    const float ey = cy + r * std::sin(a);
                    g.fillEllipse(ex - 2.0F, ey - 2.0F, 4.0F, 4.0F);
                }
                break;
            }
            case rectai::ModuleKind::kController: {
                const float innerR = iconRadius * 0.4F;
                const float outerR = iconRadius * 0.8F;
                g.drawEllipse(cx - innerR, cy - innerR, innerR * 2.0F,
                              innerR * 2.0F, 1.2F);
                g.drawEllipse(cx - outerR, cy - outerR, outerR * 2.0F,
                              outerR * 2.0F, 1.2F);
                break;
            }
            default:
                break;
            }
        }

        // Label.
        g.setColour(juce::Colours::white);
        g.setFont(14.0F);
        g.drawText(object.logical_id(),
                   juce::Rectangle<float>(cx - nodeRadius, cy - nodeRadius,
                                          nodeRadius * 2.0F,
                                          nodeRadius * 2.0F),
                   juce::Justification::centredBottom, false);
    }

    // ---------------------------------------------------------------------
    // Sequencer widget and radial menu overlays.
    // ---------------------------------------------------------------------
    for (const auto& entry : objects) {
        const auto& object = entry.second;
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

        // Radial menu for selected object.
        if (entry.first == selectedObjectId_) {
            const float menuRadius = nodeRadius + 30.0F;

            struct SliderDesc {
                float angle;
                const char* paramName;
                const char* label;
            };

            static constexpr SliderDesc kSliders[] = {
                {-juce::MathConstants<float>::halfPi, "freq", "Freq"},
                {-juce::MathConstants<float>::halfPi +
                     juce::MathConstants<float>::pi * 2.0F / 3.0F,
                 "gain", "Gain"},
                {-juce::MathConstants<float>::halfPi -
                     juce::MathConstants<float>::pi * 2.0F / 3.0F,
                 "fx", "FX"},
            };

            auto drawSlider = [&](float angle, const juce::String& label,
                                   float value) {
                const float innerR = menuRadius;
                const float outerR = menuRadius + 18.0F;

                const float sx1 = cx + innerR * std::cos(angle);
                const float sy1 = cy + innerR * std::sin(angle);
                const float sx2 = cx + outerR * std::cos(angle);
                const float sy2 = cy + outerR * std::sin(angle);

                juce::Line<float> sliderLine(sx1, sy1, sx2, sy2);
                g.setColour(juce::Colours::white.withAlpha(0.4F));
                g.drawLine(sliderLine, 2.0F);

                const float handleR =
                    juce::jmap(value, 0.0F, 1.0F, innerR, outerR);
                const float hx = cx + handleR * std::cos(angle);
                const float hy = cy + handleR * std::sin(angle);
                g.setColour(juce::Colours::white);
                g.fillEllipse(hx - 3.0F, hy - 3.0F, 6.0F, 6.0F);

                const float lx = cx + (outerR + 8.0F) * std::cos(angle);
                const float ly = cy + (outerR + 8.0F) * std::sin(angle);
                g.setFont(12.0F);
                g.drawFittedText(label, juce::Rectangle<int>(
                                                static_cast<int>(lx - 40.0F),
                                                static_cast<int>(ly - 8.0F),
                                                80, 16),
                                 juce::Justification::centred, 1);
            };

            g.setColour(juce::Colours::white.withAlpha(0.12F));
            g.fillEllipse(cx - (menuRadius + 24.0F), cy - (menuRadius + 24.0F),
                          (menuRadius + 24.0F) * 2.0F,
                          (menuRadius + 24.0F) * 2.0F);

            const auto& modulesForMenu = scene_.modules();
            const auto moduleIt =
                modulesForMenu.find(object.logical_id());

            for (const auto& slider : kSliders) {
                float value = 0.5F;
                if (moduleIt != modulesForMenu.end()) {
                    value = moduleIt->second.GetParameterOrDefault(
                        slider.paramName, 0.5F);
                }
                value = juce::jlimit(0.0F, 1.0F, value);
                drawSlider(slider.angle, slider.label, value);
            }
        }
    }
}

void MainComponent::resized()
{
}

void MainComponent::mouseDown(const juce::MouseEvent& event)
{
    const auto bounds = getLocalBounds().toFloat();
    const float radius = 30.0F;

    draggedObjectId_ = 0;
    draggingRadialSlider_ = false;
    draggingSliderIndex_ = -1;

    // First, try to select an object by clicking on its circle
    // or one of its radial sliders if a radial menu is visible.
    for (const auto& [id, object] : scene_.objects()) {
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

    // If we already have a selected object, check if the click
    // landed on one of its radial sliders; if so, start slider drag
    // instead of moving the object.
    if (selectedObjectId_ != 0) {
        const auto objects = scene_.objects();
        const auto itObj = objects.find(selectedObjectId_);
        if (itObj != objects.end()) {
            const auto& object = itObj->second;
            const auto cx = bounds.getX() + object.x() * bounds.getWidth();
            const auto cy = bounds.getY() + object.y() * bounds.getHeight();

            const float menuRadius = 26.0F + 30.0F;  // nodeRadius + offset
            const float innerR = menuRadius;
            const float outerR = menuRadius + 18.0F;

            struct SliderDesc {
                float angle;
            };

            static constexpr SliderDesc kSliderAngles[] = {
                {-juce::MathConstants<float>::halfPi},
                {-juce::MathConstants<float>::halfPi +
                 juce::MathConstants<float>::pi * 2.0F / 3.0F},
                {-juce::MathConstants<float>::halfPi -
                 juce::MathConstants<float>::pi * 2.0F / 3.0F},
            };

            const juce::Point<float> clickPos = event.position;
            constexpr float maxDistSq = 6.0F * 6.0F;

            for (int i = 0; i < 3; ++i) {
                const float angle = kSliderAngles[i].angle;
                const float sx1 = cx + innerR * std::cos(angle);
                const float sy1 = cy + innerR * std::sin(angle);
                const float sx2 = cx + outerR * std::cos(angle);
                const float sy2 = cy + outerR * std::sin(angle);

                auto isNearSegment = [](juce::Point<float> p,
                                        juce::Point<float> a,
                                        juce::Point<float> b,
                                        float maxDistSq) {
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

                if (isNearSegment(clickPos, {sx1, sy1}, {sx2, sy2},
                                  maxDistSq)) {
                    draggingRadialSlider_ = true;
                    draggingSliderIndex_ = i;
                    draggedObjectId_ = 0;  // do not move the object
                    break;
                }
            }
        }
    }

    // If no object was selected, check if the click is near a line
    // connecting an object to the centre to toggle mute.
    if (draggedObjectId_ == 0) {
        const auto centre = bounds.getCentre();
        const auto click = event.position;
        constexpr float maxDistance = 6.0F;
        const float maxDistanceSq = maxDistance * maxDistance;

        const auto& objects = scene_.objects();

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

            if (isConnectionGeometricallyActive(fromObjIt->second,
                                                toObjIt->second)) {
                objectsWithOutgoingActiveConnection.insert(fromIdIt->second);
            }
        }

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

        for (const auto& [id, object] : objects) {
            if (objectsWithOutgoingActiveConnection.find(id) !=
                objectsWithOutgoingActiveConnection.end()) {
                // This object's output is currently routed through another
                // instrument, so its direct line to master is inactive and
                // cannot be muted.
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
    } else if (!draggingRadialSlider_) {
        // Clicking directly on a node also toggles its selection for the
        // radial menu overlay.
        if (selectedObjectId_ == draggedObjectId_) {
            selectedObjectId_ = 0;
        } else {
            selectedObjectId_ = draggedObjectId_;
        }
    }
}

void MainComponent::mouseDrag(const juce::MouseEvent& event)
{
    const auto bounds = getLocalBounds().toFloat();

    if (draggingRadialSlider_ && selectedObjectId_ != 0 &&
        draggingSliderIndex_ >= 0 && draggingSliderIndex_ < 3) {
        const auto objects = scene_.objects();
        const auto itObj = objects.find(selectedObjectId_);
        if (itObj == objects.end()) {
            return;
        }

        const auto& object = itObj->second;
        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        const float menuRadius = 26.0F + 30.0F;
        const float innerR = menuRadius;
        const float outerR = menuRadius + 18.0F;

        static constexpr float sliderAngles[3] = {
            -juce::MathConstants<float>::halfPi,
            -juce::MathConstants<float>::halfPi +
                juce::MathConstants<float>::pi * 2.0F / 3.0F,
            -juce::MathConstants<float>::halfPi -
                juce::MathConstants<float>::pi * 2.0F / 3.0F,
        };

        const float angle = sliderAngles[draggingSliderIndex_];

        const float vx = static_cast<float>(event.position.x) - cx;
        const float vy = static_cast<float>(event.position.y) - cy;
        const float proj = vx * std::cos(angle) + vy * std::sin(angle);

        const float clampedR = juce::jlimit(innerR, outerR, proj);
        const float value = (clampedR - innerR) / (outerR - innerR);

        const char* paramNames[3] = {"freq", "gain", "fx"};

        scene_.SetModuleParameter(object.logical_id(),
                                  paramNames[draggingSliderIndex_], value);

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

    auto updated = it->second;
    updated.set_position(normX, normY);
    scene_.UpsertObject(updated);

    repaint();
}

void MainComponent::mouseUp(const juce::MouseEvent&)
{
    draggedObjectId_ = 0;
    draggingRadialSlider_ = false;
    draggingSliderIndex_ = -1;
}

void MainComponent::timerCallback()
{
    // Map scene state to audio parameters: sound flows either directly
    // from the oscillator to the master or through the filter, depending
    // on the active spatial connection.
    const auto& objects = scene_.objects();
    auto oscIt = std::find_if(objects.begin(), objects.end(),
                              [](const auto& pair) {
                                  return pair.second.logical_id() == "osc1";
                              });

    float targetLevel = 0.0F;

    if (oscIt != objects.end()) {
        const auto& oscObject = oscIt->second;
        const std::int64_t oscObjectId = oscIt->first;
        const float x = juce::jlimit(0.0F, 1.0F, oscObject.x());

        // Look for a filter object that could be downstream from the
        // oscillator according to the spatial connection rule.
        auto filterIt = std::find_if(objects.begin(), objects.end(),
                                     [](const auto& pair) {
                                         return pair.second.logical_id() ==
                                                "filter1";
                                     });

        bool hasActiveConnectionToFilter = false;
        std::int64_t filterObjectId = 0;
        if (filterIt != objects.end()) {
            const auto& filterObject = filterIt->second;
            filterObjectId = filterIt->first;
            hasActiveConnectionToFilter =
                isConnectionGeometricallyActive(oscObject, filterObject);
        }

        // Decide which line to master the sound is using and whether the
        // whole chain is muted.
        bool chainMuted = false;
        if (!hasActiveConnectionToFilter || filterObjectId == 0) {
            // Oscillator is loose: it connects directly to the master.
            const bool lineMuted =
                mutedObjects_.find(oscObjectId) != mutedObjects_.end();
            chainMuted = lineMuted;
        } else {
            // Oscillator feeds the filter; the filter's line to master
            // controls the entire chain.
            const bool lineMuted =
                mutedObjects_.find(filterObjectId) != mutedObjects_.end();
            chainMuted = lineMuted;
        }

        // Read UI parameters from the associated modules (radial menus).
        const float oscFreqParam = scene_.GetModuleParameterOrDefault(
            "osc1", "freq", 0.5F);

        float gainParam = scene_.GetModuleParameterOrDefault(
            "osc1", "gain", 0.5F);
        if (hasActiveConnectionToFilter && filterObjectId != 0) {
            // When the filter is in the chain, use its Gain parameter as
            // the main loudness control.
            gainParam = scene_.GetModuleParameterOrDefault(
                "filter1", "gain", gainParam);
        }

        // Position X still influences the base frequency, but the
        // oscillator's "Freq" slider scales it, giving tangible + UI
        // control.
        const double baseFreq = 200.0 + static_cast<double>(x) * 800.0;
        const double frequency = baseFreq * (0.5 + 1.0 * oscFreqParam);
        audioEngine_.setFrequency(frequency);

        // Gain slider(s) control the output level envelope.
        const float baseLevel = 0.02F;
        const float extra = 0.18F * gainParam;
        targetLevel = chainMuted ? 0.0F : (baseLevel + extra);
    }

    audioEngine_.setLevel(targetLevel);

    // Update BPM pulse animation.
    const double fps = 60.0;  // Timer frequency.
    const double dt = 1.0 / fps;

    // Age existing pulses and remove the ones that have fully faded.
    constexpr double pulseLifetimeSeconds = 1.0;
    const float ageStep = static_cast<float>(dt / pulseLifetimeSeconds);

    for (auto& pulse : pulses_) {
        pulse.age += ageStep;
    }

    pulses_.erase(std::remove_if(pulses_.begin(), pulses_.end(),
                                 [](const Pulse& p) { return p.age >= 1.0F; }),
                  pulses_.end());

    // Spawn a new pulse on every beat; every 4th beat is stronger.
    const double bps = bpm_ / 60.0;  // beats per second
    beatPhase_ += bps / fps;
    if (beatPhase_ >= 1.0) {
        beatPhase_ -= 1.0;

        const bool strong = (beatIndex_ % 4 == 0);
        pulses_.push_back(Pulse{0.0F, strong});

        ++beatIndex_;
        if (beatIndex_ >= 4) {
            beatIndex_ = 0;
        }
    }

    // Advance connection flow phase (used for pulses along edges).
    connectionFlowPhase_ += dt;
    if (connectionFlowPhase_ > 1.0) {
        connectionFlowPhase_ -= std::floor(connectionFlowPhase_);
    }

    // Simple sequencer phase for widgets (steps per bar = 8).
    const int stepsPerBar = 8;
    const double stepsPerSecond = bps * static_cast<double>(stepsPerBar);
    sequencerPhase_ += stepsPerSecond / fps;
    if (sequencerPhase_ >= 1.0) {
        sequencerPhase_ -= 1.0;
    }
    const int newStep =
        static_cast<int>(sequencerPhase_ * static_cast<double>(stepsPerBar));
    if (newStep != sequencerStep_) {
        sequencerStep_ = newStep;
    }

    repaint();
}
