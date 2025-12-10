#include "MainComponent.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

#include "AudioEngine.h"
#include "core/AudioModules.h"
#include "core/ReactableRtpLoader.h"

namespace {

juce::Colour colourFromArgb(const uint32_t argb)
{
    const auto alpha = static_cast<juce::uint8>((argb >> 24U) & 0xFFU);
    const auto red = static_cast<juce::uint8>((argb >> 16U) & 0xFFU);
    const auto green = static_cast<juce::uint8>((argb >> 8U) & 0xFFU);
    const auto blue = static_cast<juce::uint8>(argb & 0xFFU);
    // juce::Colour stores colours internally as ARGB; this constructor
    // expects components in that order.
    return juce::Colour(alpha, red, green, blue);
}

std::string makeConnectionKey(const rectai::Connection& c)
{
    std::string key;
    key.reserve(c.from_module_id.size() + c.from_port_name.size() +
                c.to_module_id.size() + c.to_port_name.size() + 4U);
    key.append(c.from_module_id)
        .append(1U, ':')
        .append(c.from_port_name)
        .append(1U, '>')
        .append(c.to_module_id)
        .append(1U, ':')
        .append(c.to_port_name);
    return key;
}

std::string makeObjectPairKey(std::int64_t a, std::int64_t b)
{
    if (a > b) {
        std::swap(a, b);
    }
    std::string key;
    key.reserve(32U);
    key.append(std::to_string(a)).append(1U, '>').append(
        std::to_string(b));
    return key;
}

std::string makeModulePairKey(const std::string& fromId,
                              const std::string& toId)
{
    std::string key;
    key.reserve(fromId.size() + toId.size() + 1U);
    key.append(fromId).append(1U, '>').append(toId);
    return key;
}

// Returns true if `toObj` lies inside a 120º cone (not visible) whose
// vertex is at the centre of the table and whose axis points towards
// `fromObj`.
bool isConnectionGeometricallyActive(const rectai::ObjectInstance& fromObj,
                                     const rectai::ObjectInstance& toObj)
{
    constexpr float centreX = 0.5F;
    constexpr float centreY = 0.5F;

    const float fromDx = fromObj.x() - centreX;
    const float fromDy = fromObj.y() - centreY;
    const float toDx = toObj.x() - centreX;
    const float toDy = toObj.y() - centreY;

    if (std::abs(fromDx) < 1.0e-4F && std::abs(fromDy) < 1.0e-4F) {
        return false;
    }
    if (std::abs(toDx) < 1.0e-4F && std::abs(toDy) < 1.0e-4F) {
        return false;
    }

    const float fromAngle = std::atan2(fromDy, fromDx);
    const float toAngle = std::atan2(toDy, toDx);

    float diff = toAngle - fromAngle;
    const float pi = juce::MathConstants<float>::pi;
    const float twoPi = juce::MathConstants<float>::twoPi;
    while (diff > pi) {
        diff -= twoPi;
    }
    while (diff < -pi) {
        diff += twoPi;
    }

    const float halfCone = juce::MathConstants<float>::pi / 3.0F;  // 120º cone
    return std::abs(diff) <= halfCone;
}

}  // namespace

bool MainComponent::isInsideMusicArea(const rectai::ObjectInstance& obj) const
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

MainComponent::MainComponent(AudioEngine& audioEngine)
    : audioEngine_(audioEngine)
{
    setSize(1280, 720);

    // Load default Reactable patch from com.reactable/Resources/default.rtp.
    // If loading fails (e.g., when running tests without assets), fall back
    // to the small hardcoded demo scene used originally.
    const auto loadDefaultPatch = [this]() {
        using rectai::LoadReactablePatchFromFile;
        using rectai::ReactablePatchMetadata;

        // Try a few likely base directories relative to the current working
        // directory and to the executable location.
        juce::File candidates[8];
        int candidateCount = 0;

        const juce::File cwd = juce::File::getCurrentWorkingDirectory();
        candidates[candidateCount++] = cwd.getChildFile(
            "com.reactable/Resources/default.rtp");
        candidates[candidateCount++] = cwd.getChildFile(
            "../com.reactable/Resources/default.rtp");
        candidates[candidateCount++] = cwd.getChildFile(
            "../../com.reactable/Resources/default.rtp");

        const juce::File exeDir = juce::File::getSpecialLocation(
                                       juce::File::currentExecutableFile)
                                       .getParentDirectory();
        candidates[candidateCount++] = exeDir.getChildFile(
            "com.reactable/Resources/default.rtp");
        candidates[candidateCount++] = exeDir.getChildFile(
            "../com.reactable/Resources/default.rtp");
        candidates[candidateCount++] = exeDir.getChildFile(
            "../../com.reactable/Resources/default.rtp");
        candidates[candidateCount++] = exeDir.getParentDirectory().getChildFile(
            "com.reactable/Resources/default.rtp");
        candidates[candidateCount++] = exeDir.getParentDirectory().getChildFile(
            "../com.reactable/Resources/default.rtp");

        for (int i = 0; i < candidateCount; ++i) {
            const juce::File& f = candidates[i];
            if (!f.existsAsFile()) {
                continue;
            }

            rectai::ReactablePatchMetadata metadata;
            std::string error;

            scene_ = rectai::Scene{};
            const bool ok = LoadReactablePatchFromFile(
                f.getFullPathName().toStdString(), scene_, &metadata, &error);
            if (ok) {
                return true;
            }
        }

        return false;
    };

    if (!loadDefaultPatch()) {
        // Fallback: example scene with a couple of modules and objects.
        auto osc1 = std::make_unique<rectai::OscillatorModule>("osc1");
        auto filter1 =
            std::make_unique<rectai::FilterModule>("filter1");

        (void)scene_.AddModule(std::move(osc1));
        (void)scene_.AddModule(std::move(filter1));

        rectai::Connection connection{
            .from_module_id = "osc1",
            .from_port_name = "out",
            .to_module_id = "filter1",
            .to_port_name = "in",
        };
        (void)scene_.AddConnection(connection);

        // Normalized positions on the table.
        scene_.UpsertObject(
            rectai::ObjectInstance(1, "osc1", 0.3F, 0.5F, 0.0F));
        scene_.UpsertObject(rectai::ObjectInstance(
            2, "filter1", 0.7F, 0.5F, 0.0F));
    }

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
        // -----------------------------------------------------------------
        const float baseRadius = 6.0F;
        g.setColour(juce::Colours::white);
        g.fillEllipse(centre.x - baseRadius, centre.y - baseRadius,
                      baseRadius * 2.0F, baseRadius * 2.0F);

        for (const auto& pulse : pulses_) {
            const float t = juce::jlimit(0.0F, 1.0F, pulse.age);
            const float maxRadius = pulse.strong ? 90.0F : 50.0F;
            const float radius = baseRadius + t * maxRadius;
            const float alpha = 1.0F - t;
            const float thickness = pulse.strong ? 5.0F : 2.0F;

            g.setColour(juce::Colours::white.withAlpha(alpha));
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
                connectionFlowPhase_ + 0.25 * (id % 4), 1.0));
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

        // Waveform / icon inside the node body depending on metadata.
        std::string iconId;
        if (moduleForObject != nullptr) {
            iconId = moduleForObject->icon_id();
        }

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
                const float s = std::sin(t * juce::MathConstants<float>::twoPi);
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
                const float hFactor = 0.3F + 0.1F * static_cast<float>(i);
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
        }

        // Label.
        juce::String labelText(object.logical_id());
        if (moduleForObject != nullptr) {
            const auto& moduleLabel = moduleForObject->label();
            if (!moduleLabel.empty()) {
                labelText = moduleLabel + " (" + object.logical_id() + ")";
            }
        }
        g.setColour(juce::Colours::white);
        g.setFont(14.0F);
        g.drawText(labelText,
                   juce::Rectangle<float>(cx - nodeRadius, cy - nodeRadius,
                                          nodeRadius * 2.0F,
                                          nodeRadius * 2.0F),
                   juce::Justification::centredBottom, false);
    }

    // ---------------------------------------------------------------------
    // Sequencer widget overlays.
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
    sideControlObjectId_ = 0;
    sideControlKind_ = SideControlKind::kNone;

    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    // First, try to grab one of the per-instrument side controls
    // (left: Freq, right: Gain) by clicking near its handle.
    const float nodeRadius = 26.0F;
    const float ringRadius = nodeRadius + 10.0F;
    const float sliderMargin = 6.0F;

    for (const auto& [id, object] : objects) {
        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

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
            gainValue = moduleForObject->GetParameterOrDefault(
                "gain",
                moduleForObject->default_parameter_value("gain"));
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

        const auto click = event.position;
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

    // If no object was selected, check if the click is near a line
    // connecting an object to the centre to toggle that instrument's mute,
    // or near a connection between instruments to toggle the source's mute.
    if (draggedObjectId_ == 0) {
        const auto centre = bounds.getCentre();
        const auto click = event.position;
        constexpr float maxDistance = 6.0F;
        const float maxDistanceSq = maxDistance * maxDistance;

        const auto& objectsLocal = scene_.objects();

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

        // Fallback: muting via the line centre → object.
        for (const auto& [id, object] : objectsLocal) {
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

        const float nodeRadius = 26.0F;
        const float ringRadius = nodeRadius + 10.0F;
        const float sliderMargin = 6.0F;
        const float sliderTop = cy - ringRadius + sliderMargin;
        const float sliderBottom = cy + ringRadius - sliderMargin;

        const float mouseY = static_cast<float>(event.position.y);
        const float clampedY = juce::jlimit(sliderTop, sliderBottom, mouseY);
        const float value = juce::jmap(clampedY, sliderBottom, sliderTop,
                                       0.0F, 1.0F);

        if (sideControlKind_ == SideControlKind::kFreq) {
            scene_.SetModuleParameter(object.logical_id(), "freq", value);
        } else if (sideControlKind_ == SideControlKind::kGain) {
            scene_.SetModuleParameter(object.logical_id(), "gain", value);
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
    updated.set_position(normX, normY);
    scene_.UpsertObject(updated);

    repaint();
}

void MainComponent::mouseUp(const juce::MouseEvent&)
{
    draggedObjectId_ = 0;
    sideControlObjectId_ = 0;
    sideControlKind_ = SideControlKind::kNone;
}

void MainComponent::toggleHardlinkBetweenObjects(const std::int64_t objectIdA,
                                                 const std::int64_t objectIdB)
{
    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    const auto itA = objects.find(objectIdA);
    const auto itB = objects.find(objectIdB);
    if (itA == objects.end() || itB == objects.end()) {
        return;
    }

    const auto& objA = itA->second;
    const auto& objB = itB->second;

    const auto modItA = modules.find(objA.logical_id());
    const auto modItB = modules.find(objB.logical_id());
    if (modItA == modules.end() || modItB == modules.end() ||
        modItA->second == nullptr || modItB->second == nullptr) {
        return;
    }

    auto* moduleA = modItA->second.get();
    auto* moduleB = modItB->second.get();

    // Decide connection direction based on existing connection policies.
    std::string fromId;
    std::string toId;

    if (moduleA->CanConnectTo(*moduleB)) {
        fromId = moduleA->id();
        toId = moduleB->id();
    } else if (moduleB->CanConnectTo(*moduleA)) {
        fromId = moduleB->id();
        toId = moduleA->id();
    } else {
        // No valid audio routing between these modules.
        return;
    }

    // Look for an existing connection between these two modules using the
    // standard audio port names.
    const auto& connections = scene_.connections();
    bool found = false;
    bool isHardlink = false;
    for (const auto& c : connections) {
        if (c.from_module_id == fromId && c.to_module_id == toId &&
            c.from_port_name == "out" && c.to_port_name == "in") {
            found = true;
            isHardlink = c.is_hardlink;
            break;
        }
    }

    if (!found) {
        // No existing connection: create a new hardlink.
        rectai::Connection connection{.from_module_id = fromId,
                                      .from_port_name = "out",
                                      .to_module_id = toId,
                                      .to_port_name = "in",
                                      .is_hardlink = true};
        (void)scene_.AddConnection(connection);
        return;
    }

    if (isHardlink) {
        // Existing hardlink: remove it. If this pair had a dynamic
        // connection that was previously promoted to hardlink, restore
        // that dynamic connection instead of leaving it disconnected.
        (void)scene_.RemoveConnection(fromId, "out", toId, "in");

        const std::string pairKey = makeModulePairKey(fromId, toId);
        const auto promotedIt = promotedHardlinkPairs_.find(pairKey);
        if (promotedIt != promotedHardlinkPairs_.end()) {
            promotedHardlinkPairs_.erase(promotedIt);

            rectai::Connection restored{.from_module_id = fromId,
                                        .from_port_name = "out",
                                        .to_module_id = toId,
                                        .to_port_name = "in",
                                        .is_hardlink = false};
            (void)scene_.AddConnection(restored);
        }
        return;
    }

    // Existing non-hardlink connection: promote it to hardlink and
    // remember that this pair had a base dynamic connection so that we
    // can restore it when toggling the hardlink off again.
    const std::string pairKey = makeModulePairKey(fromId, toId);
    promotedHardlinkPairs_.insert(pairKey);
    (void)scene_.RemoveConnection(fromId, "out", toId, "in");
    rectai::Connection upgraded{.from_module_id = fromId,
                                .from_port_name = "out",
                                .to_module_id = toId,
                                .to_port_name = "in",
                                .is_hardlink = true};
    (void)scene_.AddConnection(upgraded);
}

void MainComponent::timerCallback()
{
    // Map scene state to audio parameters using AudioModule metadata.
    // Multiple generator modules can be active at once; we currently
    // mix them into a single voice 0 for simplicity.
    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    // ------------------------------------------------------------------
    // Hardlink maintenance and collision-based toggling.
    // ------------------------------------------------------------------
    {
        const auto bounds = getLocalBounds().toFloat();
        // Virtual collision radius used to trigger hardlinks. We treat
        // nodes as if they had radius 30 px and require at least 4 px of
        // overlap between those virtual circles before considering it a
        // "strong" collision that toggles a hardlink.
        const float collisionRadius = 30.0F;
        const float maxDist = 2.0F * collisionRadius - 4.0F;  // >=4 px overlap
        const float maxDistSq = maxDist * maxDist;

        // Objects currently inside the musical area.
        std::vector<std::pair<std::int64_t, const rectai::ObjectInstance*>>
            inside;
        inside.reserve(objects.size());
        for (const auto& [objId, obj] : objects) {
            if (isInsideMusicArea(obj)) {
                inside.emplace_back(objId, &obj);
            }
        }

        // Detect new collisions between objects (touching circles) and
        // toggle hardlinks on each new contact event.
        std::unordered_set<std::string> currentPairs;
        for (std::size_t i = 0; i < inside.size(); ++i) {
            const auto idA = inside[i].first;
            const auto* objA = inside[i].second;
            const float ax = bounds.getX() + objA->x() * bounds.getWidth();
            const float ay = bounds.getY() + objA->y() * bounds.getHeight();

            for (std::size_t j = i + 1; j < inside.size(); ++j) {
                const auto idB = inside[j].first;
                const auto* objB = inside[j].second;
                const float bx =
                    bounds.getX() + objB->x() * bounds.getWidth();
                const float by =
                    bounds.getY() + objB->y() * bounds.getHeight();

                const float dx = bx - ax;
                const float dy = by - ay;
                const float distSq = dx * dx + dy * dy;
                if (distSq > maxDistSq) {
                    continue;
                }

                const std::string pairKey = makeObjectPairKey(idA, idB);
                currentPairs.insert(pairKey);

                if (activeHardlinkCollisions_.find(pairKey) ==
                    activeHardlinkCollisions_.end()) {
                    // New collision event between these two objects:
                    // toggle the hardlink connection if allowed.
                    toggleHardlinkBetweenObjects(idA, idB);
                    activeHardlinkCollisions_.insert(pairKey);
                }
            }
        }

        // Clear pairs that are no longer colliding so that a future
        // contact between them can toggle the hardlink again.
        for (auto it = activeHardlinkCollisions_.begin();
             it != activeHardlinkCollisions_.end();) {
            if (currentPairs.find(*it) == currentPairs.end()) {
                it = activeHardlinkCollisions_.erase(it);
            } else {
                ++it;
            }
        }

        // Remove hardlink connections whose endpoints are outside the
        // musical area.
        std::unordered_map<std::string, std::int64_t> moduleToObjectId;
        for (const auto& [objId, obj] : objects) {
            moduleToObjectId.emplace(obj.logical_id(), objId);
        }

        std::vector<std::pair<std::string, std::string>> hardlinksToRemove;
        for (const auto& conn : scene_.connections()) {
            if (!conn.is_hardlink) {
                continue;
            }

            const auto fromObjIdIt =
                moduleToObjectId.find(conn.from_module_id);
            const auto toObjIdIt =
                moduleToObjectId.find(conn.to_module_id);

            const rectai::ObjectInstance* fromObj = nullptr;
            const rectai::ObjectInstance* toObj = nullptr;
            if (fromObjIdIt != moduleToObjectId.end()) {
                const auto it = objects.find(fromObjIdIt->second);
                if (it != objects.end()) {
                    fromObj = &it->second;
                }
            }
            if (toObjIdIt != moduleToObjectId.end()) {
                const auto it = objects.find(toObjIdIt->second);
                if (it != objects.end()) {
                    toObj = &it->second;
                }
            }

            if (fromObj == nullptr || toObj == nullptr ||
                !isInsideMusicArea(*fromObj) ||
                !isInsideMusicArea(*toObj)) {
                hardlinksToRemove.emplace_back(conn.from_module_id,
                                               conn.to_module_id);

                // Clear any collision tracking for this pair.
                if (fromObj != nullptr && toObj != nullptr) {
                    const auto pairKey = makeObjectPairKey(
                        moduleToObjectId[conn.from_module_id],
                        moduleToObjectId[conn.to_module_id]);
                    activeHardlinkCollisions_.erase(pairKey);
                }
            }
        }

        for (const auto& ids : hardlinksToRemove) {
            (void)scene_.RemoveConnection(ids.first, "out", ids.second,
                                          "in");
        }
    }

    double mixedFrequency = 0.0;
    float mixedLevel = 0.0F;
    bool hasAnyActive = false;

    // Precompute a lookup from module id to object tracking id, so we can
    // quickly test mute/position for downstream modules.
    std::unordered_map<std::string, std::int64_t> moduleToObjectId;
    for (const auto& [objId, obj] : objects) {
        moduleToObjectId.emplace(obj.logical_id(), objId);
    }

    for (const auto& [objId, obj] : objects) {
        const auto modIt = modules.find(obj.logical_id());
        if (modIt == modules.end() || modIt->second == nullptr) {
            continue;
        }

        const auto* module = modIt->second.get();
        if (module->type() != rectai::ModuleType::kGenerator) {
            continue;
        }

        if (!isInsideMusicArea(obj)) {
            continue;
        }

        const bool srcMuted =
            mutedObjects_.find(objId) != mutedObjects_.end();

        // Try to find a downstream audio module that this generator feeds
        // according to spatial rules and connection state.
        const rectai::AudioModule* downstreamModule = nullptr;
        std::int64_t downstreamObjId = 0;
        bool downstreamInside = false;
        bool connectionMuted = false;

        for (const auto& conn : scene_.connections()) {
            if (conn.from_module_id != module->id()) {
                continue;
            }

            const auto toObjIdIt = moduleToObjectId.find(conn.to_module_id);
            if (toObjIdIt == moduleToObjectId.end()) {
                continue;
            }

            const auto objIt = objects.find(toObjIdIt->second);
            if (objIt == objects.end()) {
                continue;
            }

            const auto& toObj = objIt->second;
            if (!isInsideMusicArea(toObj) ||
                (!conn.is_hardlink &&
                 !isConnectionGeometricallyActive(obj, toObj))) {
                continue;
            }

            const auto modDestIt = modules.find(conn.to_module_id);
            if (modDestIt == modules.end() || modDestIt->second == nullptr) {
                continue;
            }

            downstreamModule = modDestIt->second.get();
            downstreamObjId = objIt->first;
            downstreamInside = true;

            const std::string key = makeConnectionKey(conn);
            connectionMuted =
                mutedConnections_.find(key) != mutedConnections_.end();
            break;
        }

        bool chainMuted = srcMuted;
        float gainParam = module->GetParameterOrDefault(
            "gain", module->default_parameter_value("gain"));

        if (downstreamModule != nullptr && downstreamInside) {
            const bool dstMuted =
                mutedObjects_.find(downstreamObjId) !=
                mutedObjects_.end();
            chainMuted = srcMuted || dstMuted || connectionMuted;
            gainParam = downstreamModule->GetParameterOrDefault(
                "gain", gainParam);
        }

        const float freqParam = module->GetParameterOrDefault(
            "freq", module->default_parameter_value("freq"));
        const double frequency =
            module->base_frequency_hz() +
            module->frequency_range_hz() * static_cast<double>(freqParam);

        const float baseLevel = module->base_level();
        const float extra = module->level_range() * gainParam;
        const float level = chainMuted ? 0.0F : (baseLevel + extra);

        if (level > 0.0F) {
            if (!hasAnyActive) {
                mixedFrequency = frequency;
                mixedLevel = level;
                hasAnyActive = true;
            } else {
                // Simple averaging when multiple generators are active.
                mixedFrequency = 0.5 * (mixedFrequency + frequency);
                mixedLevel = std::min(1.0F, mixedLevel + level);
            }
        }
    }

    if (!hasAnyActive) {
        mixedFrequency = 0.0;
        mixedLevel = 0.0F;
    }

    audioEngine_.setFrequency(mixedFrequency);
    audioEngine_.setLevel(mixedLevel);

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
