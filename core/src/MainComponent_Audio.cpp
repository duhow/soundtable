#include "MainComponent.h"

#include <algorithm>
#include <cmath>

#include "AudioEngine.h"
#include "MainComponentHelpers.h"
#include "core/AudioModules.h"
#include "core/MidiTypes.h"

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

void MainComponent::triggerSampleplayNotesOnBeat(const bool strongBeat)
{
    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    // Global master volume derived from the Volume module, if present.
    float globalVolumeParam = 1.0F;
    for (const auto& [id, modulePtr] : modules) {
        if (modulePtr == nullptr) {
            continue;
        }

        const auto* volumeModule =
            dynamic_cast<const rectai::VolumeModule*>(modulePtr.get());
        if (volumeModule != nullptr) {
            globalVolumeParam =
                volumeModule->GetParameterOrDefault("volume", 0.9F);
            break;
        }
    }

    float globalVolumeGain = 1.0F;
    if (globalVolumeParam <= 0.0F) {
        globalVolumeGain = 0.0F;
    } else if (globalVolumeParam < 1.0F) {
        const float db = -40.0F * (1.0F - globalVolumeParam);
        const float linear = std::pow(10.0F, db / 20.0F);
        globalVolumeGain = linear;
    }

    for (const auto& [objId, obj] : objects) {
        const auto modIt = modules.find(obj.logical_id());
        if (modIt == modules.end() || modIt->second == nullptr) {
            continue;
        }

        const auto* module = modIt->second.get();
        const auto* sampleModule =
            dynamic_cast<const rectai::SampleplayModule*>(module);
        if (sampleModule == nullptr) {
            continue;
        }

        if (!isInsideMusicArea(obj)) {
            continue;
        }

        const bool muted =
            mutedObjects_.find(objId) != mutedObjects_.end();
        if (muted) {
            continue;
        }

        const auto* activeInst = sampleModule->active_instrument();
        if (activeInst == nullptr) {
            continue;
        }

        // Derive MIDI note from the `midifreq` parameter as before.
        const float midiNote = sampleModule->GetParameterOrDefault(
            "midifreq", 87.0F);
        const int midiKey = juce::jlimit(
            0, 127,
            static_cast<int>(std::lround(static_cast<double>(midiNote))));

        const float ampParam = sampleModule->GetParameterOrDefault(
            "amp", 1.0F);
        if (ampParam <= 0.0F) {
            continue;
        }

        const float baseLevel = sampleModule->base_level();
        const float extra = sampleModule->level_range() * ampParam;
        float chainLevel =
            (ampParam <= 0.0F) ? 0.0F : (baseLevel + extra);

        // Apply master volume and a small accent on strong beats.
        float velocity01 = chainLevel * globalVolumeGain;
        if (strongBeat) {
            velocity01 *= 1.1F;
        }

        if (velocity01 <= 0.0F) {
            continue;
        }

        velocity01 = juce::jlimit(0.0F, 1.0F, velocity01);

        audioEngine_.triggerSampleplayNote(activeInst->bank,
                                           activeInst->program,
                                           midiKey, velocity01);

        // Mark the module as carrying audio for visual purposes so
        // its connections can be highlighted, even though the
        // underlying waveform comes from FluidSynth rather than the
        // internal oscillators.
        modulesWithActiveAudio_.insert(sampleModule->id());
    }
}

void MainComponent::timerCallback()
{
    // Map scene state to audio parameters using AudioModule metadata.
    // Multiple generator modules can be active at once; we map each
    // active generator to an independent AudioEngine voice so that
    // multiple oscillators can sound simultaneously.
    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    // ------------------------------------------------------------------
    // Rotation tracking shared by frequency and tempo controllers.
    // We keep a per-object map of the last known angle in degrees and
    // derive a wrapped delta in [-180, 180] every frame so that
    // crossing the 0/360 boundary does not create large jumps.
    const float radToDeg = 180.0F / juce::MathConstants<float>::pi;

    std::unordered_map<std::int64_t, float> rotationDeltaDegrees;
    rotationDeltaDegrees.reserve(objects.size());

    for (const auto& [objId, obj] : objects) {
        const float currentDeg = obj.angle_radians() * radToDeg;

        float diff = 0.0F;
        const auto lastIt = lastObjectAngleDegrees_.find(objId);
        if (lastIt != lastObjectAngleDegrees_.end()) {
            diff = currentDeg - lastIt->second;

            while (diff > 180.0F) {
                diff -= 360.0F;
            }
            while (diff < -180.0F) {
                diff += 360.0F;
            }
        }

        rotationDeltaDegrees.emplace(objId, diff);
        lastObjectAngleDegrees_[objId] = currentDeg;
    }

    // Drop entries for objects that are no longer present in the
    // scene to keep the tracking map bounded.
    for (auto it = lastObjectAngleDegrees_.begin();
         it != lastObjectAngleDegrees_.end();) {
        if (objects.find(it->first) == objects.end()) {
            it = lastObjectAngleDegrees_.erase(it);
        } else {
            ++it;
        }
    }

    // ------------------------------------------------------------------
    // Rotation 3 frequency mapping.
    // For modules that expose frequency control and have a tangible
    // object on the table, use their per-frame rotation delta to add a
    // normalised contribution to the module's `freq` parameter. A full
    // revolution corresponds to a ffreq of 71.0.
    for (const auto& [objId, obj] : objects) {
        const auto modIt = modules.find(obj.logical_id());
        if (modIt == modules.end() || modIt->second == nullptr) {
            continue;
        }

        auto* module = modIt->second.get();
        if (!module->uses_frequency_control()) {
            continue;
        }

        // Ignore docked modules (toolbar/dock area), but allow
        // rotation-based modulation anywhere else on the table so
        // that fiducials just outside the musical circle still
        // influence their associated module.
        if (obj.docked()) {
            continue;
        }

        const auto deltaIt = rotationDeltaDegrees.find(objId);
        if (deltaIt == rotationDeltaDegrees.end()) {
            continue;
        }

        const float diff = deltaIt->second;

        // Invert sign so that counter-clockwise rotation reduces the
        // frequency parameter and clockwise rotation increases it.
        const float deltaFreq = -diff / 360.0F;  // [-0.5, 0.5]

        if (deltaFreq == 0.0F) {
            continue;
        }

        const float currentFreq = module->GetParameterOrDefault(
            "freq", module->default_parameter_value("freq"));
        const float newFreq = juce::jlimit(0.0F, 1.0F,
                                           currentFreq + deltaFreq);

        scene_.SetModuleParameter(obj.logical_id(), "freq", newFreq);
    }

    // ------------------------------------------------------------------
    // Rotation 3 tempo (global BPM) mapping.
    // The Tempo module is treated as a global transport controller:
    // every 5 b0 of rotation increase or decrease 1 BPM. The value is
    // stored as a double in `bpm_` but rendered as an integer.
    for (const auto& [objId, obj] : objects) {
        const auto modIt = modules.find(obj.logical_id());
        if (modIt == modules.end() || modIt->second == nullptr) {
            continue;
        }

        auto* module = modIt->second.get();
        const auto* tempoModule =
            dynamic_cast<const rectai::TempoModule*>(module);
        if (tempoModule == nullptr) {
            continue;
        }

        const auto deltaIt = rotationDeltaDegrees.find(objId);
        if (deltaIt == rotationDeltaDegrees.end()) {
            continue;
        }

        const float diff = deltaIt->second;
        if (diff == 0.0F) {
            continue;
        }

        // Match the control sense used for frequency: inverting the
        // sign keeps clockwise/counter-clockwise behaviour consistent
        // across controllers.
        const double deltaBpm = static_cast<double>(-diff / 5.0F);
        if (deltaBpm == 0.0) {
            continue;
        }

        double newBpm = bpm_ + deltaBpm;
        if (newBpm < 40.0) {
            newBpm = 40.0;
        } else if (newBpm > 400.0) {
            newBpm = 400.0;
        }

        bpm_ = newBpm;

        // Remember when the BPM was last modified so that the visual
        // BPM label can be shown briefly and then faded out when there
        // are no further tempo changes.
        bpmLastChangeSeconds_ =
            juce::Time::getMillisecondCounterHiRes() / 1000.0;

        // Keep the logical Tempo module parameter in sync with the
        // session BPM so that future serialisation and loaders can
        // observe the updated tempo.
        scene_.SetModuleParameter(module->id(), "tempo",
                                  static_cast<float>(bpm_));
    }

    // ------------------------------------------------------------------
    // Rotation → global volume (master) mapping.
    // Volume modules act as session-wide output controllers: their
    // rotation gesture only modifies the `volume` parameter, leaving
    // dynamics/FX parameters untouched.
    for (const auto& [objId, obj] : objects) {
        const auto modIt = modules.find(obj.logical_id());
        if (modIt == modules.end() || modIt->second == nullptr) {
            continue;
        }

        auto* module = modIt->second.get();
        auto* volumeModule =
            dynamic_cast<rectai::VolumeModule*>(module);
        if (volumeModule == nullptr) {
            continue;
        }

        const auto deltaIt = rotationDeltaDegrees.find(objId);
        if (deltaIt == rotationDeltaDegrees.end()) {
            continue;
        }

        const float diff = deltaIt->second;
        if (diff == 0.0F) {
            continue;
        }

        // Match the control sense used for frequency and tempo so
        // clockwise/counter-clockwise behaviour remains consistent.
        const float deltaVolume = -diff / 360.0F;  // [-0.5, 0.5]
        if (deltaVolume == 0.0F) {
            continue;
        }

        const float currentVolume = volumeModule->GetParameterOrDefault(
            "volume", 0.9F);
        const float newVolume = juce::jlimit(0.0F, 1.0F,
                                             currentVolume + deltaVolume);

        scene_.SetModuleParameter(volumeModule->id(), "volume",
                                  newVolume);
    }

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

    // ------------------------------------------------------------------
    // Dynamic connection creation based on spatial layout.
    // ------------------------------------------------------------------
    {
        // Consider only objects that are currently inside the musical
        // area; docked modules and objects outside the circle are
        // ignored for dynamic connections.
        std::vector<std::pair<std::int64_t, const rectai::ObjectInstance*>>
            inside;
        inside.reserve(objects.size());
        for (const auto& [objId, obj] : objects) {
            if (isInsideMusicArea(obj)) {
                inside.emplace_back(objId, &obj);
            }
        }

        const auto& existingConnections = scene_.connections();

        // For each unordered pair of objects inside the musical area,
        // check whether a valid audio connection should exist according
        // to module policies and the geometric cone. If so, ensure that
        // a non-hardlink connection (standard out->in) is present.
        for (std::size_t i = 0; i < inside.size(); ++i) {
            const auto* objA = inside[i].second;

            const auto modItA = modules.find(objA->logical_id());
            if (modItA == modules.end() || modItA->second == nullptr) {
                continue;
            }
            auto* moduleA = modItA->second.get();

            for (std::size_t j = i + 1; j < inside.size(); ++j) {
                const auto* objB = inside[j].second;

                const auto modItB = modules.find(objB->logical_id());
                if (modItB == modules.end() || modItB->second == nullptr) {
                    continue;
                }
                auto* moduleB = modItB->second.get();

                // Decide connection direction based on existing
                // connection policies.
                std::string fromId;
                std::string toId;
                const rectai::ObjectInstance* fromObj = nullptr;
                const rectai::ObjectInstance* toObj = nullptr;

                if (moduleA->CanConnectTo(*moduleB)) {
                    fromId = moduleA->id();
                    toId = moduleB->id();
                    fromObj = objA;
                    toObj = objB;
                } else if (moduleB->CanConnectTo(*moduleA)) {
                    fromId = moduleB->id();
                    toId = moduleA->id();
                    fromObj = objB;
                    toObj = objA;
                } else {
                    // No valid audio routing between these modules.
                    continue;
                }

                // Respect the geometric cone for dynamic (non-hardlink)
                // connections. Hardlinks remain always active regardless
                // of this predicate.
                if (!isConnectionGeometricallyActive(*fromObj, *toObj)) {
                    continue;
                }

                // Skip if a connection already exists between these
                // modules using the standard audio ports.
                bool alreadyConnected = false;
                for (const auto& conn : existingConnections) {
                    if (conn.from_module_id == fromId &&
                        conn.to_module_id == toId &&
                        conn.from_port_name == "out" &&
                        conn.to_port_name == "in") {
                        alreadyConnected = true;
                        break;
                    }
                }

                if (alreadyConnected) {
                    continue;
                }

                rectai::Connection connection{.from_module_id = fromId,
                                              .from_port_name = "out",
                                              .to_module_id = toId,
                                              .to_port_name = "in",
                                              .is_hardlink = false};
                (void)scene_.AddConnection(connection);
            }
        }
    }

    // Precompute a lookup from module id to object tracking id, so we can
    // quickly test mute/position for downstream modules.
    std::unordered_map<std::string, std::int64_t> moduleToObjectId;
    for (const auto& [objId, obj] : objects) {
        moduleToObjectId.emplace(obj.logical_id(), objId);
    }

    modulesWithActiveAudio_.clear();
    moduleVoiceIndex_.clear();

    // Global master volume derived from the Volume module, if present.
    float globalVolumeParam = 1.0F;
    for (const auto& [id, modulePtr] : modules) {
        if (modulePtr == nullptr) {
            continue;
        }

        const auto* volumeModule =
            dynamic_cast<const rectai::VolumeModule*>(modulePtr.get());
        if (volumeModule != nullptr) {
            globalVolumeParam =
                volumeModule->GetParameterOrDefault("volume", 0.9F);
            break;
        }
    }

    // Map the normalised volume control [0,1] to a perceptual
    // gain curve in dB so that el volumen por defecto no resulte
    // excesivo y los cambios pequeños alrededor del 90% sean más
    // progresivos.
    float globalVolumeGain = 1.0F;
    if (globalVolumeParam <= 0.0F) {
        globalVolumeGain = 0.0F;
    } else if (globalVolumeParam >= 1.0F) {
        globalVolumeGain = 1.0F;
    } else {
        const float minDb = -40.0F;
        const float db = -40.0F * (1.0F - globalVolumeParam);
        const float linear = std::pow(10.0F, db / 20.0F);
        globalVolumeGain = linear;
    }

    // Compute a global gain for Sampleplay (SoundFont) output. Since
    // all Sampleplay modules share a single synthesiser instance in
    // the AudioEngine, we can only mute/stop SoundFont audio
    // globally. The rule used here is: if the master is muted, or if
    // there is no Sampleplay module inside the musical area with its
    // own line to the master unmuted, then the Sampleplay path gain
    // is set to 0 (hard stop). Otherwise it follows the master
    // volume.
    float sampleplayOutputGain = masterMuted_ ? 0.0F : globalVolumeGain;
    if (!masterMuted_) {
        bool hasUnmutedSampleplay = false;
        for (const auto& [objId, obj] : objects) {
            const auto modIt = modules.find(obj.logical_id());
            if (modIt == modules.end() || modIt->second == nullptr) {
                continue;
            }

            auto* module = modIt->second.get();
            if (dynamic_cast<rectai::SampleplayModule*>(module) == nullptr) {
                continue;
            }

            if (!isInsideMusicArea(obj)) {
                continue;
            }

            const bool isMutedObject =
                mutedObjects_.find(objId) != mutedObjects_.end();
            if (!isMutedObject) {
                hasUnmutedSampleplay = true;
                break;
            }
        }

        if (!hasUnmutedSampleplay) {
            sampleplayOutputGain = 0.0F;
        }
    }

    audioEngine_.setSampleplayOutputGain(sampleplayOutputGain);

    struct VoiceParams {
        double frequency{0.0};
        float level{0.0F};
        int waveform{0};
    };

    VoiceParams voices[AudioEngine::kMaxVoices];
    int voiceIndex = 0;

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
        std::string downstreamConnectionKey;

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
            downstreamConnectionKey = key;
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

            // For most audio modules, let the downstream module's gain
            // control the chain level. Filters are treated specially:
            // their right-hand control represents resonance (Q), not
            // output volume, so it should not affect the overall level
            // used here.
            if (downstreamModule->type() != rectai::ModuleType::kFilter) {
                gainParam = downstreamModule->GetParameterOrDefault(
                    "gain", gainParam);
            }
        }

        const float freqParam = module->GetParameterOrDefault(
            "freq", module->default_parameter_value("freq"));
        const double frequency =
            module->base_frequency_hz() +
            module->frequency_range_hz() * static_cast<double>(freqParam);

        const float baseLevel = module->base_level();
        const float extra = module->level_range() * gainParam;
        // Ensure that a gain parameter of 0.0 corresponds to true
        // silence, rather than the minimum base level.
        const float calculatedLevel =
            (gainParam <= 0.0F) ? 0.0F : (baseLevel + extra);
        
        // Check if this chain is being held for temporary mute visualization.
        // During hold, we still want to process audio (for waveform capture)
        // but with zero output level.
        const bool isBeingHeld = activeConnectionHold_.has_value() &&
                                 ((activeConnectionHold_->is_object_line && 
                                   activeConnectionHold_->object_id == objId) ||
                                  (!activeConnectionHold_->is_object_line && 
                                   downstreamModule != nullptr && downstreamInside &&
                                   downstreamConnectionKey == activeConnectionHold_->connection_key));

        int waveformIndex = 0;  // 0 = sine by default.
        if (const auto* oscModule =
                dynamic_cast<const rectai::OscillatorModule*>(module)) {
            using Waveform = rectai::OscillatorModule::Waveform;
            const auto wf = oscModule->waveform();
            if (wf == Waveform::kSaw) {
                waveformIndex = 1;
            } else if (wf == Waveform::kSquare) {
                waveformIndex = 2;
            } else if (wf == Waveform::kNoise) {
                waveformIndex = 3;
            } else {
                waveformIndex = 0;
            }
        }

        // Process voice if level would be > 0 OR if it's being held for visualization.
        if ((calculatedLevel > 0.0F || isBeingHeld) && voiceIndex < AudioEngine::kMaxVoices) {
            voices[voiceIndex].frequency = frequency;
            // Output level: 0 if muted/held/master-muted, otherwise normal level.
            const float outputLevel = (chainMuted || isBeingHeld || masterMuted_) 
                                         ? 0.0F 
                                         : (calculatedLevel * globalVolumeGain);
            voices[voiceIndex].level = outputLevel;
            voices[voiceIndex].waveform = waveformIndex;
            const int assignedVoice = voiceIndex;
            ++voiceIndex;

            // Configure optional per-voice filter when the generator
            // feeds a FilterModule. The filter cutoff is controlled
            // by the filter module's own `freq` parameter (left bar),
            // and resonance by its `q` parameter (right bar). For
            // now we always use the low-pass mode, but other modes
            // are implemented in the audio engine for future use.
            int filterMode = 0;
            double filterCutoffHz = 0.0;
            float filterQ = 0.7071F;

            if (downstreamModule != nullptr && downstreamInside &&
                downstreamModule->type() == rectai::ModuleType::kFilter) {
                const auto* filterModule =
                    dynamic_cast<const rectai::FilterModule*>(
                        downstreamModule);

                const float filterFreqParam =
                    downstreamModule->GetParameterOrDefault(
                        "freq", downstreamModule->default_parameter_value(
                                    "freq"));
                const double fb = downstreamModule->base_frequency_hz();
                const double fr = downstreamModule->frequency_range_hz();
                filterCutoffHz = fb + fr *
                                        static_cast<double>(filterFreqParam);

                const float qParam = downstreamModule->GetParameterOrDefault(
                    "q", downstreamModule->default_parameter_value("q"));
                const float minQ = 0.5F;
                const float maxQ = 10.0F;
                filterQ = minQ + (maxQ - minQ) * qParam;

                // Map FilterModule::Mode to AudioEngine filter mode.
                if (filterModule != nullptr) {
                    using Mode = rectai::FilterModule::Mode;
                    const auto mode = filterModule->mode();
                    if (mode == Mode::kLowPass) {
                        filterMode = 1;
                    } else if (mode == Mode::kBandPass) {
                        filterMode = 2;
                    } else if (mode == Mode::kHighPass) {
                        filterMode = 3;
                    }
                } else {
                    // Default to low-pass if we do not know the mode.
                    filterMode = 1;
                }
            }

            audioEngine_.setVoiceFilter(assignedVoice, filterMode,
                                        filterCutoffHz, filterQ);
            audioEngine_.setVoiceWaveform(assignedVoice, waveformIndex);

            // Mark generator and, when present, its downstream module
            // as actively carrying audio so the visual layer can
            // render waveforms only on those paths, and remember
            // which AudioEngine voice index represents this chain.
            modulesWithActiveAudio_.insert(module->id());
            moduleVoiceIndex_[module->id()] = assignedVoice;
            if (downstreamModule != nullptr && downstreamInside &&
                !connectionMuted) {
                modulesWithActiveAudio_.insert(
                    downstreamModule->id());
                moduleVoiceIndex_[downstreamModule->id()] = assignedVoice;
            }
        }
    }

    // Apply per-voice state to the AudioEngine. Any voices beyond the
    // active count are explicitly silenced so that oscillators removed
    // from the scene stop producing sound.
    for (int v = 0; v < AudioEngine::kMaxVoices; ++v) {
        if (v < voiceIndex) {
            audioEngine_.setVoice(v, voices[v].frequency, voices[v].level);
        } else {
            audioEngine_.setVoice(v, 0.0, 0.0F);
        }
    }

    // Update BPM pulse animation using real dt between timer ticks so
    // visuals remain stable even if the timer frequency changes or the
    // event loop hiccups.
    const double nowSeconds =
        juce::Time::getMillisecondCounterHiRes() / 1000.0;
    double dt = nowSeconds - lastTimerSeconds_;
    if (dt <= 0.0 || dt > 0.5) {
        // Fallback to a small, reasonable timestep in case of the
        // first frame or large pauses (e.g. debugger).
        dt = 1.0 / 120.0;
    }
    lastTimerSeconds_ = nowSeconds;

    // Age existing pulses and remove the ones that have fully faded.
    constexpr double pulseLifetimeSeconds = 1.0;
    const float ageStep = static_cast<float>(dt / pulseLifetimeSeconds);

    for (auto& pulse : pulses_) {
        pulse.age += ageStep;
    }

    pulses_.erase(std::remove_if(pulses_.begin(), pulses_.end(),
                                 [](const Pulse& p) { return p.age >= 1.0F; }),
                  pulses_.end());

    // Helper to run one audio step de todos los módulos Sequencer
    // para un índice de step dado, generando un MidiNoteEvent por
    // step activo y dejando que los módulos Sampleplay/Oscillator
    // consuman ese evento como entrada lógica común.
    auto runSequencerStep = [&](const int stepIndex) {
        const auto& modules = scene_.modules();
        const auto& objects = scene_.objects();

        // Precompute mapping from module id to object id so we can
        // test spatial predicates and mute state for destinations.
        std::unordered_map<std::string, std::int64_t> moduleToObjectId;
        moduleToObjectId.reserve(objects.size());
        for (const auto& [objId, obj] : objects) {
            moduleToObjectId.emplace(obj.logical_id(), objId);
        }

        if (stepIndex < 0 ||
            stepIndex >= rectai::SequencerPreset::kNumSteps) {
            return;
        }

        for (const auto& [id, modulePtr] : modules) {
            if (modulePtr == nullptr) {
                continue;
            }

            auto* seqModule =
                dynamic_cast<rectai::SequencerModule*>(modulePtr.get());
            if (seqModule == nullptr) {
                continue;
            }

            // Require the Sequencer tangible to be present and
            // inside the musical area.
            const auto objIdIt = moduleToObjectId.find(id);
            if (objIdIt == moduleToObjectId.end()) {
                continue;
            }

            const auto objIt = objects.find(objIdIt->second);
            if (objIt == objects.end()) {
                continue;
            }

            const auto& obj = objIt->second;
            if (!isInsideMusicArea(obj)) {
                continue;
            }

            // Skip muted Sequencer objects.
            if (mutedObjects_.find(objIdIt->second) !=
                mutedObjects_.end()) {
                continue;
            }

            // Resolve current preset and step.
            const int presetIndex = seqModule->current_preset();
            if (presetIndex < 0 ||
                presetIndex >= rectai::SequencerModule::kNumPresets) {
                continue;
            }

            const auto& preset = seqModule->preset(presetIndex);
            const auto& step =
                preset.steps[static_cast<std::size_t>(stepIndex)];

            // Si el paso está desactivado, no disparamos notas y,
            // opcionalmente, forzamos silencio explícito en los
            // osciladores conectados solo cuando el Sequencer está
            // autorizado a controlar volumen.
            if (!step.enabled) {
                if (sequencerControlsVolume_) {
                    for (const auto& conn : scene_.connections()) {
                        if (conn.from_module_id != id) {
                            continue;
                        }

                        const auto modDestIt =
                            modules.find(conn.to_module_id);
                        if (modDestIt == modules.end() ||
                            modDestIt->second == nullptr) {
                            continue;
                        }

                        if (auto* oscModule =
                                dynamic_cast<rectai::OscillatorModule*>(
                                    modDestIt->second.get())) {
                            scene_.SetModuleParameter(oscModule->id(),
                                                      "gain", 0.0F);
                        }
                    }
                }
                continue;
            }

            // For each Sequencer, walk outgoing connections and
            // drive downstream modules that can consume what the
            // Sequencer produces (Sampleplay/Oscillator for ahora).
            for (const auto& conn : scene_.connections()) {
                if (conn.from_module_id != id) {
                    continue;
                }

                const auto toObjIdIt =
                    moduleToObjectId.find(conn.to_module_id);
                if (toObjIdIt == moduleToObjectId.end()) {
                    continue;
                }

                const auto dstObjIt = objects.find(toObjIdIt->second);
                if (dstObjIt == objects.end()) {
                    continue;
                }

                const auto& dstObj = dstObjIt->second;

                // Respect musical area and geometric cone for
                // non-hardlink connections.
                if (!isInsideMusicArea(dstObj) ||
                    (!conn.is_hardlink &&
                     !isConnectionGeometricallyActive(obj, dstObj))) {
                    continue;
                }

                const auto modDestIt = modules.find(conn.to_module_id);
                if (modDestIt == modules.end() ||
                    modDestIt->second == nullptr) {
                    continue;
                }

                auto* dstModule = modDestIt->second.get();

                // Estado de mute del destino y de la conexión.
                const bool dstMuted =
                    mutedObjects_.find(toObjIdIt->second) !=
                    mutedObjects_.end();
                const std::string connKey =
                    makeConnectionKey(conn);
                const bool connectionMuted =
                    mutedConnections_.find(connKey) !=
                    mutedConnections_.end();

                // Si la conexión está silenciada, no propagamos
                // nada. El estado mute del destino sólo se tiene en
                // cuenta en ramas donde realmente queremos apagar el
                // audio (Sampleplay); para Oscillator seguimos
                // enviando "MIDI" (cambios de freq/gain) aunque el
                // módulo esté silenciado, tal y como se controla en
                // la etapa de mezcla.
                if (connectionMuted) {
                    continue;
                }

                // Construimos una descripción MIDI canónica para el
                // step actual. El runtime sigue siendo monofónico, de
                // modo que solo usamos la nota principal, pero el
                // vector SequencerStep::pitches ya está preparado para
                // modos futuros.
                rectai::MidiNoteEvent noteEvent;
                noteEvent.channel = 0;  // único canal lógico por ahora.
                noteEvent.note = juce::jlimit(0, 127, step.pitch);
                noteEvent.velocity01 =
                    juce::jlimit(0.0F, 1.0F, step.velocity01);
                noteEvent.timeBeats = transportBeats_;
                noteEvent.is_note_on = true;

                // 1) Sampleplay: trigger una nota corta usando el
                // MidiNoteEvent junto con el estado de nivel del
                // módulo y el volumen global.
                if (const auto* sampleModule =
                        dynamic_cast<const rectai::SampleplayModule*>(
                            dstModule)) {
                    if (dstMuted) {
                        continue;
                    }
                    const auto* activeInst =
                        sampleModule->active_instrument();
                    if (activeInst == nullptr) {
                        continue;
                    }

                    const float ampParam =
                        sampleModule->GetParameterOrDefault(
                            "amp", 1.0F);
                    if (ampParam <= 0.0F) {
                        continue;
                    }

                    const float baseLevel = sampleModule->base_level();
                    const float extra =
                        sampleModule->level_range() * ampParam;
                    float chainLevel =
                        (ampParam <= 0.0F)
                            ? 0.0F
                            : (baseLevel + extra);

                    const float stepVelocity = sequencerControlsVolume_
                                                    ? noteEvent.velocity01
                                                    : 1.0F;

                    float effectiveVelocity = chainLevel * globalVolumeGain *
                                             stepVelocity;
                    if (effectiveVelocity <= 0.0F) {
                        continue;
                    }

                    effectiveVelocity = juce::jlimit(0.0F, 1.0F,
                                                     effectiveVelocity);

                    audioEngine_.triggerSampleplayNote(
                        activeInst->bank, activeInst->program,
                        noteEvent.note, effectiveVelocity);

                    modulesWithActiveAudio_.insert(
                        sampleModule->id());
                    continue;
                }

                // 2) Oscillator: actualiza freq/gain a partir del
                // MidiNoteEvent; el motor de audio recogerá los
                // cambios en el siguiente tick.
                if (auto* oscModule = dynamic_cast<rectai::OscillatorModule*>(
                        dstModule)) {
                    const double targetHz = 440.0 *
                                             std::pow(2.0,
                                                      (static_cast<double>(
                                                           noteEvent.note) -
                                                       69.0) /
                                                          12.0);

                    const double baseHz = oscModule->base_frequency_hz();
                    const double rangeHz =
                        oscModule->frequency_range_hz();
                    if (rangeHz > 0.0) {
                        const double norm =
                            (targetHz - baseHz) / rangeHz;
                        const float freqParam = juce::jlimit(
                            0.0F, 1.0F,
                            static_cast<float>(norm));
                        scene_.SetModuleParameter(oscModule->id(),
                                                  "freq", freqParam);
                    }

                    if (sequencerControlsVolume_) {
                        const float gainParam = juce::jlimit(
                            0.0F, 1.0F, noteEvent.velocity01);
                        scene_.SetModuleParameter(oscModule->id(), "gain",
                                                  gainParam);
                    }
                }
            }
        }
    };

    // Spawn a new pulse on every beat; every 4th beat is stronger.
    const double bps = bpm_ / 60.0;  // beats per second
    beatPhase_ += bps * dt;
    if (beatPhase_ >= 1.0) {
        beatPhase_ -= 1.0;

        // Avanza un contador simple de beats para etiquetar
        // MidiNoteEvent::timeBeats con una posición en beats
        // monotónica, sin introducir todavía un scheduler MIDI
        // completo.
        transportBeats_ += 1.0;

        const bool strong = (beatIndex_ % 4 == 0);
        pulses_.push_back(Pulse{0.0F, strong});

        // Trigger Sampleplay notes in sync with the global
        // transport using FluidSynth.
        triggerSampleplayNotesOnBeat(strong);

        // Advance Sequencer audio step: one step per beat so that
        // presets like the demo C-major scale follow the tempo
        // claramente (una nota por beat).
        const int audioStepsPerBar = rectai::SequencerPreset::kNumSteps;
        if (audioStepsPerBar > 0) {
            sequencerAudioStep_ =
                (sequencerAudioStep_ + 1) % audioStepsPerBar;
            runSequencerStep(sequencerAudioStep_);
        }

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
    sequencerPhase_ += stepsPerSecond * dt;
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
