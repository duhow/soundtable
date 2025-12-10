#include "MainComponent.h"

#include <algorithm>

#include "AudioEngine.h"
#include "MainComponentHelpers.h"

using rectai::ui::isConnectionGeometricallyActive;
using rectai::ui::makeConnectionKey;
using rectai::ui::makeModulePairKey;
using rectai::ui::makeObjectPairKey;

void MainComponent::toggleHardlinkBetweenObjects(
    const std::int64_t objectIdA, const std::int64_t objectIdB)
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

    if (masterMuted_) {
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
