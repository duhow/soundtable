#include "MainComponent.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "AudioEngine.h"
#include "MainComponentHelpers.h"
#include "core/AudioModules.h"
#include "core/MidiTypes.h"

using rectai::ui::isConnectionGeometricallyActive;
using rectai::ui::makeConnectionKey;
using rectai::ui::makeModulePairKey;
using rectai::ui::makeObjectPairKey;
using rectai::ui::generateConnectionFromModules;

#include <optional>

namespace {

// Map Reactable Sequencer `speed` values (with `speed_type="binary"`)
// to note lengths expressed in beats, assuming a 4/4 meter where a
// quarter note is one beat. This matches the original table:
//   0 = 1/32, 1 = 1/16, 2 = 1/8, 3 = 1/4, 4 = 2/4, 5 = 4/4.
[[nodiscard]] double binarySpeedToBeats(const int speed)
{
    // Clamp speed to the supported range [0,5].
    const int clamped = std::clamp(speed, 0, 5);
    switch (clamped) {
    case 0:
        return 1.0 / 8.0;   // 1/32 note = 0.125 beats.
    case 1:
        return 1.0 / 4.0;   // 1/16 note = 0.25 beats.
    case 2:
        return 1.0 / 2.0;   // 1/8 note = 0.5 beats.
    case 3:
        return 1.0;         // 1/4 note = 1 beat.
    case 4:
        return 2.0;         // 2/4 note = 2 beats.
    case 5:
    default:
        return 4.0;         // 4/4 note = 4 beats.
    }
}

}  // namespace

void MainComponent::refreshInsideMusicAreaFlags()
{
    // Take a snapshot so we can upsert safely while iterating.
    const auto objectsSnapshot = scene_.objects();
    for (const auto& [id, obj] : objectsSnapshot) {
        juce::ignoreUnused(id);
        auto updated = obj;
        const bool inside = computeInsideMusicArea(updated);
        updated.set_inside_music_area(inside);
        scene_.UpsertObject(updated);
    }
}

void MainComponent::toggleHardlinkBetweenObjects(
    const std::int64_t objectIdA, const std::int64_t objectIdB)
{
    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    // Keep the engine's internal logical audio graph in sync with
    // the current Scene so that audio-side structures (such as
    // connection waveform taps or future DSP routing) can reason
    // about modules and typed connections without re-parsing the
    // Scene.
    audioEngine_.rebuildAudioGraphFromScene(scene_);

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

    // Prepare the connection object for lookup or creation.
    rectai::Connection connection;

    // No valid audio routing between these modules.
    if(!generateConnectionFromModules(*moduleA, *moduleB, true, connection)) {
        return;
    }

    // Look for an existing connection between these two modules using the
    // standard audio port names.
    const auto& connections = scene_.connections();
    bool found = false;
    bool isHardlink = false;
    for (const auto& c : connections) {
        if (c == connection) {
            found = true;
            isHardlink = c.is_hardlink;
            break;
        }
    }

    if (!found) {
        // No existing connection: create a new hardlink.
        (void)scene_.AddConnection(connection);
        return;
    }

    const std::string pairKey = makeModulePairKey(connection);

    if (isHardlink) {
        // Existing hardlink: remove it. If this pair had a dynamic
        // connection that was previously promoted to hardlink, restore
        // that dynamic connection instead of leaving it disconnected.
        (void)scene_.RemoveConnection(connection);
        const auto promotedIt = promotedHardlinkPairs_.find(pairKey);
        if (promotedIt != promotedHardlinkPairs_.end()) {
            promotedHardlinkPairs_.erase(promotedIt);

            connection.is_hardlink = false;
            (void)scene_.AddConnection(connection);
        }
        return;
    }

    // Existing non-hardlink connection: promote it to hardlink and
    // remember that this pair had a base dynamic connection so that we
    // can restore it when toggling the hardlink off again.
    promotedHardlinkPairs_.insert(pairKey);
    (void)scene_.RemoveConnection(connection);
    connection.is_hardlink = true;
    (void)scene_.AddConnection(connection);
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

        // Treat the Sampleplay module as muted when all its
        // effective routes to the master Output (MASTER_OUTPUT_ID)
        // are muted at connection level. This is driven by the
        // implicit module -> MASTER_OUTPUT_ID connection created by
        // the loader.
        bool mutedToMaster = false;
        {
            bool hasMasterRoute = false;
            bool masterRouteMuted = true;

            const auto audioEdges =
                audioEngine_.audioGraph().audio_edges();

            for (const auto& edge : audioEdges) {
                if (edge.from_module_id != module->id() ||
                    edge.to_module_id != rectai::MASTER_OUTPUT_ID) {
                    continue;
                }

                hasMasterRoute = true;

                rectai::Connection conn{edge.from_module_id,
                                         edge.from_port_name,
                                         edge.to_module_id,
                                         edge.to_port_name,
                                         edge.is_hardlink};

                const std::string key = makeConnectionKey(conn);
                const bool connIsMuted =
                    mutedConnections_.find(key) !=
                    mutedConnections_.end();
                if (!connIsMuted) {
                    masterRouteMuted = false;
                    break;
                }
            }

            mutedToMaster = hasMasterRoute && masterRouteMuted;
        }

        if (mutedToMaster) {
            continue;
        }

        const auto* activeInst = sampleModule->active_instrument();
        if (activeInst == nullptr) {
            continue;
        }

        // Derive MIDI note from the `midifreq` parameter. When used
        // on Sampleplay modules, `midifreq` may be integer or
        // fractional; we explicitly floor the value so that
        // midifreq=59.9 sigue disparando la nota 59, respetando la
        // semántica de nota MIDI entera.
        const float midiNote = sampleModule->GetParameterOrDefault(
            "midifreq", 87.0F);
        const int midiKey = juce::jlimit(
            0, 127,
            static_cast<int>(std::floor(static_cast<double>(midiNote))));

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

void MainComponent::updateConnectionVisualSources()
{
    connectionVisualSources_.clear();

    const auto& modules = scene_.modules();

    // Helper to determine whether a Scene::Connection carries audio
    // by inspecting the declared port kinds on source/destination
    // modules.
    auto isAudioConnection = [&](const rectai::Connection& conn) {
        const auto fromIt = modules.find(conn.from_module_id);
        const auto toIt = modules.find(conn.to_module_id);

        const rectai::AudioModule* fromModule =
            (fromIt != modules.end() && fromIt->second != nullptr)
                ? fromIt->second.get()
                : nullptr;
        const rectai::AudioModule* toModule =
            (toIt != modules.end() && toIt->second != nullptr)
                ? toIt->second.get()
                : nullptr;

        if (fromModule != nullptr) {
            for (const auto& port : fromModule->output_ports()) {
                if (port.name == conn.from_port_name) {
                    return port.kind == rectai::PortSignalKind::kAudio;
                }
            }
        }

        if (toModule != nullptr) {
            for (const auto& port : toModule->input_ports()) {
                if (port.name == conn.to_port_name) {
                    return port.kind == rectai::PortSignalKind::kAudio;
                }
            }
        }

        // Fallback: treat the connection as audio only when both
        // modules agree that they produce/consume audio.
        if (fromModule != nullptr && toModule != nullptr) {
            return fromModule->produces_audio() &&
                   toModule->consumes_audio();
        }

        return false;
    };

    for (const auto& conn : scene_.connections()) {
        if (!isAudioConnection(conn)) {
            continue;
        }

        const auto fromIt = modules.find(conn.from_module_id);
        const auto toIt = modules.find(conn.to_module_id);

        const rectai::AudioModule* fromModule =
            (fromIt != modules.end() && fromIt->second != nullptr)
                ? fromIt->second.get()
                : nullptr;
        const rectai::AudioModule* toModule =
            (toIt != modules.end() && toIt->second != nullptr)
                ? toIt->second.get()
                : nullptr;

        if (fromModule == nullptr && toModule == nullptr) {
            continue;
        }

        ConnectionVisualSource source;

        // Sampleplay connections always use the dedicated Sampleplay
        // waveform buffer rather than any per-voice buffer.
        const bool involvesSampleplay =
            (fromModule != nullptr &&
             fromModule->is<rectai::SampleplayModule>()) ||
            (toModule != nullptr &&
             toModule->is<rectai::SampleplayModule>());

        if (involvesSampleplay) {
            source.kind = ConnectionVisualSource::Kind::kSampleplay;
            source.voiceIndex = -1;
        } else {
            int voiceIndex = -1;

            // Prefer mapping based on the source module id; fall back
            // to the destination when the source is not currently
            // mapped to any voice.
            if (const auto it =
                    moduleVoiceIndex_.find(conn.from_module_id);
                it != moduleVoiceIndex_.end()) {
                voiceIndex = it->second;
            } else if (const auto it =
                           moduleVoiceIndex_.find(conn.to_module_id);
                       it != moduleVoiceIndex_.end()) {
                voiceIndex = it->second;
            }

            if (voiceIndex < 0 || voiceIndex >= AudioEngine::kMaxVoices) {
                continue;
            }

            source.voiceIndex = voiceIndex;

            const bool fromIsGenerator =
                (fromModule != nullptr &&
                 fromModule->type() == rectai::ModuleType::kGenerator);
            const bool fromIsFilter =
                (fromModule != nullptr &&
                 fromModule->is<rectai::FilterModule>());

            // For generator→X connections use the pre-filter
            // waveform so that Osc→Filter shows the raw oscillator.
            // For all other cases (Filter outputs, FX, etc.) use the
            // post-filter waveform.
            if (fromIsGenerator && !fromIsFilter) {
                source.kind = ConnectionVisualSource::Kind::kVoicePre;
            } else {
                source.kind = ConnectionVisualSource::Kind::kVoicePost;
            }
        }

        const std::string key = makeConnectionKey(conn);
        connectionVisualSources_.emplace(key, source);
    }
}

void MainComponent::rotationTrackingUpdate(std::unordered_map<std::int64_t, float>* rotationDeltaDegrees)
{
    // ------------------------------------------------------------------
    // Rotation tracking shared by frequency and tempo controllers.
    // We keep a per-object map of the last known angle in degrees and
    // derive a wrapped delta in [-180, 180] every frame so that
    // crossing the 0/360 boundary does not create large jumps.
    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    const float radToDeg = 180.0F / juce::MathConstants<float>::pi;

    rotationDeltaDegrees->reserve(objects.size());
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

        rotationDeltaDegrees->emplace(objId, diff);
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
}

void MainComponent::timerCallback()
{
    // Keep the per-object inside-music-area flag in sync with the
    // current component bounds and object positions before running
    // any audio or visual mapping for this tick.
    refreshInsideMusicAreaFlags();

    // Map scene state to audio parameters using AudioModule metadata.
    // Multiple generator modules can be active at once; we map each
    // active generator to an independent AudioEngine voice so that
    // multiple oscillators can sound simultaneously.
    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    std::unordered_map<std::int64_t, float> rotationDeltaDegrees;
    rotationTrackingUpdate(&rotationDeltaDegrees);

    // ------------------------------------------------------------------
    // Rotation gestures → module parameters.
    // For each tangible object with a rotation delta we:
    //   - adjust `freq` on any AudioModule that uses frequency control
    //     (typical generators/filters), except when the object is docked.
    //   - adjust global tempo/BPM when the module is a TempoModule.
    //   - adjust global master volume when the module is a VolumeModule.
    // All three mappings share the same rotation sense: clockwise
    // increases the controlled value, counter-clockwise decreases it.
    for (const auto& [objId, obj] : objects) {
        const auto modIt = modules.find(obj.logical_id());
        if (modIt == modules.end() || modIt->second == nullptr) {
            continue;
        }

        auto* module = modIt->second.get();

        const auto deltaIt = rotationDeltaDegrees.find(objId);
        if (deltaIt == rotationDeltaDegrees.end()) {
            continue;
        }

        const float diff = deltaIt->second;

        // ------------------------------------------------------------------
        // 1) Rotation → per-module frequency (`freq` parameter) or
        // Loop sample selection (`sample` parameter).
        // For standard modules with frequency control, a full
        // revolution corresponds to a normalised delta of one unit.
        // For Loop modules, the same rotation scrolls over the four
        // discrete sample segments with wrap-around behaviour.
        if (auto* loopModule =
                dynamic_cast<rectai::LoopModule*>(module)) {
            if (!obj.docked()) {
                // One full revolution (360º) scrolls across the four
                // Loop segments, so every ~90º the selected slot
                // changes. We keep the underlying parameter
                // continuous in [0,1] so the UI triangle can move
                // smoothly with the fiducial rotation.
                const float deltaSample = -diff / 360.0F;  // [-0.5, 0.5]

                if (std::fabs(deltaSample) >
                    std::numeric_limits<float>::epsilon()) {
                    const float currentSampleParam =
                        loopModule->GetParameterOrDefault("sample", 0.0F);
                    float clampedCurrent = juce::jlimit(
                        0.0F, 1.0F, currentSampleParam);
                    float updated = clampedCurrent + deltaSample;

                    // Infinite scroll: when crossing the [0,1]
                    // bounds, wrap the value instead of clamping so
                    // the user can rotate continuously through the
                    // 4 segments.
                    if (updated > 1.0F) {
                        updated -= 1.0F;
                    } else if (updated < 0.0F) {
                        updated += 1.0F;
                    }

                    auto paramToIndex = [](const float v) {
                        int idx = static_cast<int>(v * 4.0F);
                        if (idx < 0) {
                            idx = 0;
                        } else if (idx > 3) {
                            idx = 3;
                        }
                        return idx;
                    };

                    const int previousIndex = paramToIndex(clampedCurrent);
                    const int newIndex = paramToIndex(updated);

                    scene_.SetModuleParameter(obj.logical_id(), "sample",
                                              updated);

                    if (newIndex != previousIndex) {
                        markLoopSampleLabelActive(loopModule->id());
                    }
                }
            }
        } else if (module->uses_frequency_control() && !obj.docked()) {
            // Invert sign so that counter-clockwise rotation reduces the
            // frequency parameter and clockwise rotation increases it.
            const float deltaFreq = -diff / 360.0F;  // [-0.5, 0.5]

            if (std::fabs(deltaFreq) <=
                std::numeric_limits<float>::epsilon()) {
                continue;
            }

            const float currentFreq = module->GetParameterOrDefault(
                "freq", module->default_parameter_value("freq"));
            const float newFreq = juce::jlimit(
                0.0F, 1.0F, currentFreq + deltaFreq);

            scene_.SetModuleParameter(obj.logical_id(), "freq",
                                      newFreq);
        }

        // ------------------------------------------------------------------
        // 2) Rotation → global tempo (BPM).
        // When the module is a TempoModule, its rotation controls the
        // session BPM: every 5 degrees of rotation add or subtract 1 BPM.
        if (auto* tempoModule =
                dynamic_cast<rectai::TempoModule*>(module)) {
            (void)tempoModule;  // type tag only; BPM lives in MainComponent.

            if (std::fabs(diff) <=
                std::numeric_limits<float>::epsilon()) {
                continue;
            }

            // Match the control sense used for frequency so
            // clockwise/counter-clockwise behaviour stays
            // consistent across controllers.
            const double deltaBpm =
                static_cast<double>(-diff / 5.0F);
            if (std::fabs(deltaBpm) <=
                std::numeric_limits<double>::epsilon()) {
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
        // 3) Rotation → global volume (master).
        // When the module is a VolumeModule, its rotation behaves as a
        // session-wide output controller that only affects the
        // normalised `volume` parameter.
        if (auto* volumeModule =
                dynamic_cast<rectai::VolumeModule*>(module)) {
            // Match the control sense used for frequency and tempo so
            // clockwise/counter-clockwise behaviour remains consistent.
            const float deltaVolume = -diff / 360.0F;  // [-0.5, 0.5]
            if (std::fabs(deltaVolume) <=
                std::numeric_limits<float>::epsilon()) {
                continue;
            }

            const float currentVolume =
                volumeModule->GetParameterOrDefault("volume", 0.9F);
            const float newVolume = juce::jlimit(0.0F, 1.0F,
                                                 currentVolume + deltaVolume);

            scene_.SetModuleParameter(volumeModule->id(), "volume",
                                      newVolume);
        }
    }

    // Keep the loop engine in sync with the current BPM so that
    // Loop modules can optionally time-stretch their playback to
    // stay on beat.
    audioEngine_.setLoopGlobalTempo(bpm_);

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
                !isInsideMusicArea(*toObj)
            ) {
                (void)scene_.RemoveConnection(conn);

                // Clear any collision tracking for this pair.
                if (fromObj != nullptr && toObj != nullptr) {
                    const auto pairKey = makeObjectPairKey(
                        moduleToObjectId[conn.from_module_id],
                        moduleToObjectId[conn.to_module_id]);
                    activeHardlinkCollisions_.erase(pairKey);
                }
            }
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

        // Score table for dynamic connections between non-generator
        // modules. For cada módulo origen conservamos solo el candidato
        // con mejor puntuación (más cercano dentro del cono).
        struct DynamicCandidate {
            rectai::Connection connection;
            float score{0.0F};
        };

        std::unordered_map<std::string, DynamicCandidate> bestByFrom;

        // For each unordered pair of objects inside the musical area,
        // compute a score for a potential audio connection according to
        // module policies, the geometric cone and distance. Only the
        // best-scoring candidate per source module will be materialised
        // as an actual dynamic connection.
        //
        // Generator modules are handled separately so that their
        // dynamic connections can follow the "closest Filter" rule
        // while still respecting the global limit of a single
        // non-hardlink outgoing connection per module.
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

                // Dynamic connections whose source or destination is a
                // GeneratorModule are handled in a dedicated pass below
                // so that generators can always target the closest
                // compatible Filter while keeping a single dynamic
                // outgoing connection. Skip those pairs here.
                if (moduleA->type() == rectai::ModuleType::kGenerator ||
                    moduleB->type() == rectai::ModuleType::kGenerator) {
                    continue;
                }

                // Decide connection direction based on existing
                // connection policies.
                rectai::Connection connection;
                if (!generateConnectionFromModules(*moduleA, *moduleB,
                                                   false, connection)) {
                    continue;
                }

                // Avoid proposing explicit dynamic connections towards
                // the invisible Output/master; esas rutas ya las
                // gestiona el loader como auto-wiring.
                if (connection.to_module_id == rectai::MASTER_OUTPUT_ID) {
                    continue;
                }

                const rectai::ObjectInstance* fromObj = nullptr;
                const rectai::ObjectInstance* toObj = nullptr;

                if (moduleA->CanConnectTo(*moduleB)) {
                    fromObj = objA;
                    toObj = objB;
                } else if (moduleB->CanConnectTo(*moduleA)) {
                    fromObj = objB;
                    toObj = objA;
                } else {
                    // No valid audio routing between these modules.
                    continue;
                }

                // Respect the geometric cone for dynamic (non-hardlink)
                // connections. Hardlinks remain active regardless of
                // this predicate.
                if (!isConnectionGeometricallyActive(*fromObj, *toObj)) {
                    continue;
                }

                // Skip if a connection already exists between these
                // modules using the standard audio ports.
                bool alreadyConnected = false;
                for (const auto& conn : existingConnections) {
                    if (conn == connection) {
                        alreadyConnected = true;
                        break;
                    }
                }

                if (alreadyConnected) {
                    continue;
                }

                // Keep the per-object inside-music-area flag in sync with the
                // current component bounds and object positions before running
                // any audio or visual mapping for this tick.
                refreshInsideMusicAreaFlags();

                // Score candidate: modules more near to each other get
                // higher score (distancia menor).
                const float dx = toObj->x() - fromObj->x();
                const float dy = toObj->y() - fromObj->y();
                const float distSq = dx * dx + dy * dy;
                const float score = -distSq;

                auto it = bestByFrom.find(connection.from_module_id);
                if (it == bestByFrom.end() || score > it->second.score) {
                    bestByFrom[connection.from_module_id] =
                        DynamicCandidate{connection, score};
                }
            }
        }

        // Materialise only the best-scoring candidate per source
        // module. Scene::AddConnection seguirá aplicando las
        // invariantes globales (máx. una conexión dinámica saliente,
        // no duplicar conexiones, etc.).
        for (const auto& [fromId, candidate] : bestByFrom) {
            (void)fromId;
            (void)scene_.AddConnection(candidate.connection);
        }
    }

    // ------------------------------------------------------------------
    // Generator dynamic connections: prefer closest Filter.
    // ------------------------------------------------------------------
    {
        const auto& connections = scene_.connections();

        // Index existing non-hardlink connections by source module so
        // we can adjust a generator's single dynamic route to its
        // closest compatible downstream module.
        std::unordered_map<std::string, std::vector<rectai::Connection>>
            dynamicByFrom;
        for (const auto& conn : connections) {
            if (conn.is_hardlink) {
                continue;
            }

            // Skip implicit/master connections.
            if (conn.to_module_id == rectai::MASTER_OUTPUT_ID) {
                continue;
            }

            dynamicByFrom[conn.from_module_id].push_back(conn);
        }

        // For each generator inside the musical area, pick the
        // closest compatible downstream module, preferring Filters
        // when available, and rewrite its single dynamic connection
        // to point to that module. Hardlinks are left untouched.
        for (const auto& [objId, obj] : objects) {
            const auto modIt = modules.find(obj.logical_id());
            if (modIt == modules.end() || modIt->second == nullptr) {
                continue;
            }

            auto* srcModule = modIt->second.get();

            // Any non-settings module (audio, generators, FX, etc.)
            // can participate in dynamic routing as long as
            // AudioModule::CanConnectTo allows it. Settings modules
            // (Tempo, Tonalizer, etc.) are excluded from this pass.
            if (srcModule->type() == rectai::ModuleType::kSettings) {
                continue;
            }

            if (!isInsideMusicArea(obj)) {
                continue;
            }

            const rectai::AudioModule* bestModule = nullptr;
            std::string bestModuleId;
            float bestDistSq = std::numeric_limits<float>::max();
            bool bestIsFilter = false;

            // Scan all other objects inside the musical area as
            // potential downstream modules.
            for (const auto& [otherObjId, otherObj] : objects) {
                if (otherObjId == objId) {
                    continue;
                }

                if (!isInsideMusicArea(otherObj)) {
                    continue;
                }

                const auto destModIt = modules.find(otherObj.logical_id());
                if (destModIt == modules.end() ||
                    destModIt->second == nullptr) {
                    continue;
                }

                auto* destModule = destModIt->second.get();

                // Ignore Output/master and global controllers as
                // downstream targets for the generator dynamic link.
                if (destModule->id() == rectai::MASTER_OUTPUT_ID ||
                    destModule->is_global_controller()) {
                    continue;
                }

                if (!srcModule->CanConnectTo(*destModule)) {
                    continue;
                }

                if (!isConnectionGeometricallyActive(obj, otherObj)) {
                    continue;
                }

                const bool destIsFilter =
                    (destModule->type() == rectai::ModuleType::kFilter);
                const float dx = otherObj.x() - obj.x();
                const float dy = otherObj.y() - obj.y();
                const float distSq = dx * dx + dy * dy;

                if (bestModule == nullptr ||
                    (destIsFilter && !bestIsFilter) ||
                    (destIsFilter == bestIsFilter &&
                     distSq < bestDistSq)) {
                    bestModule = destModule;
                    bestModuleId = destModule->id();
                    bestDistSq = distSq;
                    bestIsFilter = destIsFilter;
                }
            }

            auto dynIt = dynamicByFrom.find(srcModule->id());
            const rectai::Connection* existingConn = nullptr;
            if (dynIt != dynamicByFrom.end() && !dynIt->second.empty()) {
                existingConn = &dynIt->second.front();
            }

            // If there is no suitable downstream module in range,
            // drop any existing dynamic non-master connections from
            // this generator so that it can later reattach when a
            // compatible target becomes available.
            if (bestModule == nullptr) {
                if (dynIt != dynamicByFrom.end()) {
                    for (const auto& c : dynIt->second) {
                        (void)scene_.RemoveConnection(c);
                    }
                }
                continue;
            }

            // If the existing dynamic connection already points to
            // the best downstream module using the standard audio
            // ports, keep it.
            if (existingConn != nullptr &&
                !existingConn->is_hardlink &&
                existingConn->to_module_id == bestModuleId &&
                existingConn->from_port_name == "out" &&
                existingConn->to_port_name == "in") {
                continue;
            }

            // Remove any previous dynamic connections (non-hardlink
            // and non-master) from this generator so that
            // Scene::AddConnection can enforce the single
            // non-hardlink-outgoing invariant.
            if (dynIt != dynamicByFrom.end()) {
                for (const auto& c : dynIt->second) {
                    (void)scene_.RemoveConnection(c);
                }
            }

            rectai::Connection newConn{srcModule->id(), "out",
                                       bestModuleId, "in", false};
            (void)scene_.AddConnection(newConn);
        }
    }

    // Keep the engine's internal logical audio graph in sync with the
    // current Scene after hardlink and dynamic connection updates so
    // that downstream routing and waveform taps can rely on a
    // canonical list of typed edges.
    audioEngine_.rebuildAudioGraphFromScene(scene_);

    // Canonical edge lists derived from the current Scene snapshot.
    const auto audioEdges = audioEngine_.audioGraph().audio_edges();
    const auto& graphEdges = audioEngine_.audioGraph().edges();

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
    // gain curve in dB so that the default volume is not excessive
    // and small changes around 90% feel more progressive.
    float globalVolumeGain = 1.0F;
    if (globalVolumeParam <= 0.0F) {
        globalVolumeGain = 0.0F;
    } else if (globalVolumeParam >= 1.0F) {
        globalVolumeGain = 1.0F;
    } else {
        const float db = -40.0F * (1.0F - globalVolumeParam);
        const float linear = std::pow(10.0F, db / 20.0F);
        globalVolumeGain = linear;
    }

    // Compute a global gain for Sampleplay (SoundFont) output. Since
    // all Sampleplay modules share a single synthesiser instance in
    // the AudioEngine, we can only mute/stop SoundFont audio
    // globally. The rule used here is: if there is no Sampleplay
    // module inside the musical area with its own line to the master
    // unmuted, then the Sampleplay path gain is set to 0 (hard stop).
    // Otherwise it follows the master volume. The master visual mute
    // state (Output tangible) does not gate audio here to avoid
    // loading patches whose Output starts muted in a completely
    // silent state.
    float sampleplayOutputGain = globalVolumeGain;

    // Temporary hold-mute: if the user is holding down either the
    // Sampleplay radial to the master or one of its direct audio
    // connections, force the global Sampleplay path gain to 0 while
    // the gesture is active. This mirrors the behaviour implemented
    // for Oscillator generators, where hold-mute silences the chain
    // without modifying persistent mute state.
    bool holdBlocksSampleplay = false;
    if (activeConnectionHold_.has_value()) {
        const auto& hold = *activeConnectionHold_;

        if (hold.is_object_line) {
            const auto objIt = objects.find(hold.object_id);
            if (objIt != objects.end()) {
                const auto& obj = objIt->second;
                const auto modIt = modules.find(obj.logical_id());
                if (modIt != modules.end() &&
                    modIt->second != nullptr &&
                    modIt->second->is<rectai::SampleplayModule>()) {
                    holdBlocksSampleplay = true;
                }
            }
        } else if (!hold.connection_key.empty()) {
            for (const auto& edge : audioEdges) {
                rectai::Connection tmpConn{edge.from_module_id,
                                           edge.from_port_name,
                                           edge.to_module_id,
                                           edge.to_port_name,
                                           edge.is_hardlink};
                const std::string key = makeConnectionKey(tmpConn);
                if (key != hold.connection_key) {
                    continue;
                }

                const auto modIt = modules.find(edge.from_module_id);
                if (modIt != modules.end() &&
                    modIt->second != nullptr &&
                    modIt->second->is<rectai::SampleplayModule>()) {
                    holdBlocksSampleplay = true;
                }
                break;
            }
        }
    }

    if (!holdBlocksSampleplay) {
        // Consider the Sampleplay path muted if **all** effective
        // routes from Sampleplay modules to the master are muted at
        // connection level. A Sampleplay module is considered routed
        // to master when it has an auto-wired connection to
        // Output (-1); muting that connection is equivalent to
        // muting its radial line.
        bool hasUnmutedSampleplay = false;

        for (const auto& [objId, obj] : objects) {
            const auto modIt = modules.find(obj.logical_id());
            if (modIt == modules.end() || modIt->second == nullptr) {
                continue;
            }

            auto* module = modIt->second.get();
            if (!module->is<rectai::SampleplayModule>()) {
                continue;
            }

            if (!isInsideMusicArea(obj)) {
                continue;
            }

            // Find the implicit connection Sampleplay ->
            // Output (MASTER_OUTPUT_ID) using the audio graph so we
            // only consider audio routes.
            bool routeToMasterMuted = true;  // Assume muted until a
                                             // non-muted route is
                                             // found.
            for (const auto& edge : audioEdges) {
                if (edge.from_module_id != module->id() ||
                    edge.to_module_id != rectai::MASTER_OUTPUT_ID) {
                    continue;
                }

                rectai::Connection tmpConn{
                    edge.from_module_id,
                    edge.from_port_name,
                    edge.to_module_id,
                    edge.to_port_name,
                    edge.is_hardlink};

                const std::string key = makeConnectionKey(tmpConn);
                const bool connIsMuted =
                    mutedConnections_.find(key) !=
                    mutedConnections_.end();

                if (!connIsMuted) {
                    routeToMasterMuted = false;
                    break;
                }
            }

            if (!routeToMasterMuted) {
                hasUnmutedSampleplay = true;
                break;
            }
        }

        if (!hasUnmutedSampleplay) {
            sampleplayOutputGain = 0.0F;
        }
    } else {
        sampleplayOutputGain = 0.0F;
    }

    audioEngine_.setSampleplayOutputGain(sampleplayOutputGain);

    // Configure per-module Loop parameters (selected slot and gain)
    // based on the current Scene and routing. Loops keep running
    // even when their radial to the master is muted; here we only
    // gate the gain passed to the AudioEngine.
    for (const auto& [objId, obj] : objects) {
        juce::ignoreUnused(objId);

        const auto modIt = modules.find(obj.logical_id());
        if (modIt == modules.end() || modIt->second == nullptr) {
            continue;
        }

        auto* module = modIt->second.get();
        auto* loopModule =
            dynamic_cast<rectai::LoopModule*>(module);
        if (loopModule == nullptr) {
            continue;
        }

        // Envelope parameters for this Loop module as loaded from
        // the .rtp envelope. We pass them to the audio engine so
        // that loop gain changes can be smoothed using attack /
        // release times.
        const auto& loopEnv = loopModule->envelope();

        // Check for temporary hold-mute gestures on this Loop module.
        // If the radial from this module to the master is being held,
        // or a direct audio connection originating from this module
        // is held, we treat the loop as muted while keeping playback
        // positions advancing in the engine.
        bool isHeldForLoop = false;
        if (activeConnectionHold_.has_value()) {
            const auto& hold = *activeConnectionHold_;

            if (hold.is_object_line && hold.object_id == objId) {
                isHeldForLoop = true;
            } else if (!hold.is_object_line &&
                       !hold.connection_key.empty()) {
                for (const auto& edge : audioEdges) {
                    if (edge.from_module_id != loopModule->id()) {
                        continue;
                    }

                    rectai::Connection tmpConn{edge.from_module_id,
                                               edge.from_port_name,
                                               edge.to_module_id,
                                               edge.to_port_name,
                                               edge.is_hardlink};
                    const std::string key = makeConnectionKey(tmpConn);
                    if (key == hold.connection_key) {
                        isHeldForLoop = true;
                        break;
                    }
                }
            }
        }

        if (!isInsideMusicArea(obj)) {
            // Keep playback positions advancing in the engine but
            // treat the module as effectively silent.
            audioEngine_.setLoopModuleParams(loopModule->id(), 0,
                                             0.0F,
                                             loopEnv.attack,
                                             loopEnv.decay,
                                             loopEnv.duration,
                                             loopEnv.release);
            continue;
        }

        // Determine whether the implicit Loop → Output(master)
        // connection is muted at connection level.
        bool routeToMasterMuted = true;
        for (const auto& edge : audioEdges) {
            if (edge.from_module_id != loopModule->id() ||
                edge.to_module_id != rectai::MASTER_OUTPUT_ID) {
                continue;
            }

            rectai::Connection tmpConn{edge.from_module_id,
                                       edge.from_port_name,
                                       edge.to_module_id,
                                       edge.to_port_name,
                                       edge.is_hardlink};
            const std::string key = makeConnectionKey(tmpConn);
            const bool connIsMuted =
                mutedConnections_.find(key) !=
                mutedConnections_.end();
            if (!connIsMuted) {
                routeToMasterMuted = false;
                break;
            }
        }

        float ampParam = loopModule->GetParameterOrDefault(
            "amp", 1.0F);
        ampParam = juce::jlimit(0.0F, 1.0F, ampParam);

        // Map the Loop "amp" parameter through the same global
        // volume curve used for generators and Sampleplay.
        float loopGain = ampParam;
        if (loopGain > 0.0F) {
            const float db = -40.0F * (1.0F - loopGain);
            const float linear = std::pow(10.0F, db / 20.0F);
            loopGain = linear * globalVolumeGain;
        }

        if (routeToMasterMuted || isHeldForLoop) {
            loopGain = 0.0F;
        }

        // Derive selected slot index from the normalised "sample"
        // parameter in [0,1]. We quantise it to four segments.
        float sampleParam = loopModule->GetParameterOrDefault(
            "sample", 0.0F);
        sampleParam = juce::jlimit(0.0F, 1.0F, sampleParam);
        int selectedIndex = static_cast<int>(sampleParam * 4.0F);
        if (selectedIndex < 0) {
            selectedIndex = 0;
        } else if (selectedIndex > 3) {
            selectedIndex = 3;
        }

        audioEngine_.setLoopModuleParams(loopModule->id(),
                         selectedIndex, loopGain,
                         loopEnv.attack,
                         loopEnv.decay,
                         loopEnv.duration,
                         loopEnv.release);

        if (loopGain > 0.0F) {
            modulesWithActiveAudio_.insert(loopModule->id());
        }
    }

    // Configure an optional filter on the global Sampleplay path
    // when there is an active Sampleplay → Filter connection. Since
    // all Sampleplay modules share a single FluidSynth instance, we
    // approximate the desired routing by selecting the closest
    // compatible Filter connected to any Sampleplay module and
    // applying its parameters to a dedicated Sampleplay filter in
    // the AudioEngine.
    int sampleplayFilterMode = 0;
    double sampleplayFilterCutoffHz = 0.0;
    float sampleplayFilterQ = 0.7071F;
    {
        const rectai::FilterModule* bestFilter = nullptr;
        float bestDistSq = std::numeric_limits<float>::max();

        for (const auto& [objId, obj] : objects) {
            const auto modIt = modules.find(obj.logical_id());
            if (modIt == modules.end() || modIt->second == nullptr) {
                continue;
            }

            auto* srcModule = modIt->second.get();
            if (!srcModule->is<rectai::SampleplayModule>()) {
                continue;
            }

            if (!isInsideMusicArea(obj)) {
                continue;
            }

            // Scan audio edges for Sampleplay → Filter connections
            // that are active (inside area, respect cone for
            // dynamic links and not muted at connection level).
            for (const auto& edge : audioEdges) {
                if (edge.from_module_id != srcModule->id() ||
                    edge.to_module_id == "-1") {
                    continue;
                }

                const auto destModIt = modules.find(edge.to_module_id);
                if (destModIt == modules.end() ||
                    destModIt->second == nullptr) {
                    continue;
                }

                auto* candidateFilter =
                    dynamic_cast<rectai::FilterModule*>(
                        destModIt->second.get());
                if (candidateFilter == nullptr) {
                    continue;
                }

                const auto toObjIdIt =
                    moduleToObjectId.find(edge.to_module_id);
                if (toObjIdIt == moduleToObjectId.end()) {
                    continue;
                }

                const auto objIt = objects.find(toObjIdIt->second);
                if (objIt == objects.end()) {
                    continue;
                }

                const auto& destObj = objIt->second;
                if (!isInsideMusicArea(destObj)) {
                    continue;
                }

                if (!edge.is_hardlink &&
                    !isConnectionGeometricallyActive(obj, destObj)) {
                    continue;
                }

                rectai::Connection tmpConn{
                    edge.from_module_id,
                    edge.from_port_name,
                    edge.to_module_id,
                    edge.to_port_name,
                    edge.is_hardlink};
                const std::string key = makeConnectionKey(tmpConn);
                const bool connIsMuted =
                    mutedConnections_.find(key) !=
                    mutedConnections_.end();
                if (connIsMuted) {
                    continue;
                }

                const float dx = destObj.x() - obj.x();
                const float dy = destObj.y() - obj.y();
                const float distSq = dx * dx + dy * dy;

                if (bestFilter == nullptr || distSq < bestDistSq) {
                    bestFilter = candidateFilter;
                    bestDistSq = distSq;
                }
            }
        }

        if (bestFilter != nullptr) {
            using Mode = rectai::FilterModule::Mode;

            float filterFreqParam = bestFilter->GetParameterOrDefault(
                "freq",
                bestFilter->default_parameter_value("freq"));
            filterFreqParam =
                juce::jlimit(0.0F, 1.0F, filterFreqParam);
            const double fb = bestFilter->base_frequency_hz();
            const double fr = bestFilter->frequency_range_hz();
            sampleplayFilterCutoffHz =
                fb + fr * static_cast<double>(filterFreqParam);

            const float qParam = bestFilter->GetParameterOrDefault(
                "q",
                bestFilter->default_parameter_value("q"));
            const float minQ = 0.5F;
            const float maxQ = 10.0F;
            sampleplayFilterQ =
                minQ + (maxQ - minQ) * qParam;

            const auto mode = bestFilter->mode();
            if (mode == Mode::kLowPass) {
                sampleplayFilterMode = 1;
            } else if (mode == Mode::kBandPass) {
                sampleplayFilterMode = 2;
            } else if (mode == Mode::kHighPass) {
                sampleplayFilterMode = 3;
            } else {
                sampleplayFilterMode = 0;
            }
        }
    }

    audioEngine_.setSampleplayFilter(sampleplayFilterMode,
                                     sampleplayFilterCutoffHz,
                                     sampleplayFilterQ);

    struct VoiceParams {
        double frequency{0.0};
        float level{0.0F};
        int waveform{0};
    };

    VoiceParams voices[AudioEngine::kMaxVoices];
    int voiceIndex = 0;

    // Per-tick guard so we only emit one debug line for the first
    // active generator, keeping logs readable while still capturing
    // whether any Oscillator chain is actually producing audio.
    bool loggedGeneratorDebug = false;

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

            // A generator is considered muted at source when all its
            // effective routes to the master are muted at connection
            // level. This is modelled as the auto-wired connection from
            // the generator to the invisible Output (MASTER_OUTPUT_ID);
            // muting that connection is equivalent to muting the radial
            // line.
        bool srcMuted = false;
        {
            bool hasMasterRoute = false;
            bool masterRouteMuted = true;  // assume muted until proven otherwise

            for (const auto& edge : audioEdges) {
                if (edge.from_module_id != module->id() ||
                    edge.to_module_id != rectai::MASTER_OUTPUT_ID) {
                    continue;
                }

                hasMasterRoute = true;

                rectai::Connection tmpConn{
                    edge.from_module_id,
                    edge.from_port_name,
                    edge.to_module_id,
                    edge.to_port_name,
                    edge.is_hardlink};

                const std::string key = makeConnectionKey(tmpConn);
                const bool connIsMuted =
                    mutedConnections_.find(key) !=
                    mutedConnections_.end();
                if (!connIsMuted) {
                    masterRouteMuted = false;
                    break;
                }
            }

            // If there is an explicit route to master and it is
            // muted, treat the source as muted; otherwise it is
            // unmuted.
            srcMuted = hasMasterRoute && masterRouteMuted;
        }

        // Build all downstream routes from this generator according to
        // the current connection graph, respecting the musical area,
        // geometric cone for dynamic connections and connection-level
        // mute state. Each route will be mapped to a separate voice so
        // that hardlinks and multiple downstream modules can all carry
        // audio.
        struct GeneratorRoute {
            const rectai::AudioModule* module{nullptr};
            std::int64_t objId{0};
            bool inside{false};
            bool connectionMuted{false};
            std::string connectionKey;
        };

        std::vector<GeneratorRoute> routes;
        routes.reserve(4U);

        for (const auto& edge : audioEdges) {
            // Skip auto-wired connections from generators to the
            // invisible Output/master module (id MASTER_OUTPUT_ID).
            // The current runtime does not model the Output module as
            // an explicit processing node in the audio graph, so
            // treating it as a downstream target here would change
            // behaviour compared to scenes without auto-wiring
            // (generators direct to master).
            if (edge.to_module_id == rectai::MASTER_OUTPUT_ID) {
                continue;
            }

            if (edge.from_module_id != module->id()) {
                continue;
            }

            const auto toObjIdIt = moduleToObjectId.find(edge.to_module_id);
            if (toObjIdIt == moduleToObjectId.end()) {
                continue;
            }

            const auto objIt = objects.find(toObjIdIt->second);
            if (objIt == objects.end()) {
                continue;
            }

            const auto& toObj = objIt->second;
            if (!isInsideMusicArea(toObj)) {
                continue;
            }

            // Dynamic connections are only considered active when the
            // destination lies inside the geometric cone of the
            // source. Hardlink connections remain active regardless of
            // the cone and are treated as always-on.
            if (!edge.is_hardlink &&
                !isConnectionGeometricallyActive(obj, toObj)) {
                continue;
            }

            const auto modDestIt = modules.find(edge.to_module_id);
            if (modDestIt == modules.end() || modDestIt->second == nullptr) {
                continue;
            }

            auto* candidate = modDestIt->second.get();

            GeneratorRoute route;
            route.module = candidate;
            route.objId = objIt->first;
            route.inside = true;
            rectai::Connection tmpConn{
                edge.from_module_id,
                edge.from_port_name,
                edge.to_module_id,
                edge.to_port_name,
                edge.is_hardlink};
            route.connectionKey = makeConnectionKey(tmpConn);
            route.connectionMuted =
                mutedConnections_.find(route.connectionKey) !=
                mutedConnections_.end();
            routes.push_back(std::move(route));
        }

        // If a generator has no explicit downstream routes (for
        // example only the implicit auto-wired connection to the
        // master), we still treat it as a single direct chain so that
        // its own level and mute state drive one voice.
        if (routes.empty()) {
            routes.emplace_back();
        }

        const float freqParam = module->GetParameterOrDefault(
            "freq", module->default_parameter_value("freq"));
        const double frequency =
            module->base_frequency_hz() +
            module->frequency_range_hz() * static_cast<double>(freqParam);

        const float baseLevel = module->base_level();
        const float levelRange = module->level_range();

        int waveformIndex = 0;  // 0 = sine by default.
        const auto* oscModule =
            dynamic_cast<const rectai::OscillatorModule*>(module);
        if (oscModule != nullptr) {
            waveformIndex = oscModule->waveform_index();
        }

        for (const auto& route : routes) {
            const auto* downstreamModule = route.module;
            const bool downstreamInside = route.inside;

            bool downstreamMutedToMaster = false;

            // Base gain comes from the module's own `gain`
            // parameter (white handle). For Oscillator modules, an
            // additional Sequencer-driven gain factor may further
            // reduce the effective level, but it can never raise it
            // above the user-set value.
            float userGainParam = module->GetParameterOrDefault(
                "gain", module->default_parameter_value("gain"));
            userGainParam = juce::jlimit(0.0F, 1.0F, userGainParam);
            float gainParam = userGainParam;

            if (downstreamModule != nullptr && downstreamInside) {
                // A downstream module can also be effectively muted if
                // all of its routes to the master Output (-1) are
                // muted. When the generator feeds such a module (for
                // example two Oscillators both feeding a Filter whose
                // radial line is muted), the entire chain must be
                // considered silent even if the generator's own radial
                // is unmuted.
                {
                    bool hasMasterRoute = false;
                    bool masterRouteMuted = true;

                    for (const auto& edge : audioEdges) {
                        if (edge.from_module_id !=
                                downstreamModule->id() ||
                            edge.to_module_id != "-1") {
                            continue;
                        }

                        hasMasterRoute = true;

                        rectai::Connection tmpConn{
                            edge.from_module_id,
                            edge.from_port_name,
                            edge.to_module_id,
                            edge.to_port_name,
                            edge.is_hardlink};

                        const std::string key = makeConnectionKey(tmpConn);
                        const bool connIsMuted =
                            mutedConnections_.find(key) !=
                            mutedConnections_.end();
                        if (!connIsMuted) {
                            masterRouteMuted = false;
                            break;
                        }
                    }

                    downstreamMutedToMaster =
                        hasMasterRoute && masterRouteMuted;
                }

                // For most audio modules, let the downstream module's
                // gain control the chain level. Filters are treated
                // specially: their right-hand control represents
                // resonance (Q), not output volume, so it should not
                // affect the overall level used here.
                if (downstreamModule->type() !=
                    rectai::ModuleType::kFilter) {
                    gainParam = downstreamModule->GetParameterOrDefault(
                        "gain", gainParam);
                }
            }

            // Apply Sequencer-driven gain only for Oscillator
            // generators when allowed. The Sequencer controls a
            // separate factor in [0,1] and the final effective gain
            // is the minimum of the white handle (user gain) and
            // this factor, so that external MIDI cannot push the
            // visible module volume above the user setting.
            if (oscModule != nullptr && sequencerControlsVolume_) {
                float seqGain = 1.0F;
                const auto it = oscillatorSequencerGain_.find(module->id());
                if (it != oscillatorSequencerGain_.end()) {
                    seqGain = juce::jlimit(0.0F, 1.0F, it->second);
                }
                const float cappedUserGain = userGainParam;
                gainParam = std::min(cappedUserGain, seqGain);
            }
            bool chainMuted = srcMuted || route.connectionMuted ||
                               downstreamMutedToMaster;

            const float extra = levelRange * gainParam;
            // Ensure that a gain parameter of 0.0 corresponds to true
            // silence, rather than the minimum base level.
            const float calculatedLevel =
                (gainParam <= 0.0F) ? 0.0F : (baseLevel + extra);

            // Check if this chain is being held for temporary mute
            // visualization. During hold, we still want to process
            // audio (for waveform capture) but with zero output level.
            const bool isBeingHeld = activeConnectionHold_.has_value() &&
                                     ((activeConnectionHold_->is_object_line &&
                                       activeConnectionHold_->object_id ==
                                           objId) ||
                                      (!activeConnectionHold_->is_object_line &&
                                       !route.connectionKey.empty() &&
                                       route.connectionKey ==
                                           activeConnectionHold_
                                               ->connection_key));

            // Always allocate a voice per generator route (while
            // within the voice budget) so that the audio engine's
            // envelope can control the actual audible level,
            // including release/decay tails, even when the
            // instantaneous chain level or Sequencer-controlled
            // gain drop to zero.
            if (voiceIndex < AudioEngine::kMaxVoices) {
                voices[voiceIndex].frequency = frequency;
                // Output level: 0 if muted/held, otherwise normal
                // level.
                const float outputLevel =
                    (chainMuted || isBeingHeld)
                        ? 0.0F
                        : (calculatedLevel * globalVolumeGain);
                voices[voiceIndex].level = outputLevel;
                voices[voiceIndex].waveform = waveformIndex;

#if !defined(NDEBUG)
                // Lightweight debug log to validate generator audio
                // state without flooding the log: only log the first
                // active generator per timer tick.
                if (!loggedGeneratorDebug) {
                    juce::String msg("[rectai-core][audio-debug] gen=");
                    msg << module->id().c_str() << " freqHz="
                        << frequency << " level=" << calculatedLevel
                        << " out=" << outputLevel
                        << " muted=" << (chainMuted ? "1" : "0");
                    if (downstreamModule != nullptr && downstreamInside) {
                        msg << " downstream="
                            << downstreamModule->id().c_str();
                    } else {
                        msg << " downstream=none";
                    }
                    juce::Logger::writeToLog(msg);
                    loggedGeneratorDebug = true;
                }
#endif  // !defined(NDEBUG)

                const int assignedVoice = voiceIndex;
                ++voiceIndex;

                // Configure optional per-voice filter when the
                // generator feeds a FilterModule. The filter cutoff is
                // controlled by the filter module's own `freq`
                // parameter (left bar), and resonance by its `q`
                // parameter (right bar). For now we always use the
                // low-pass mode, but other modes are implemented in
                // the audio engine for future use.
                int filterMode = 0;
                double filterCutoffHz = 0.0;
                float filterQ = 0.7071F;

                if (downstreamModule != nullptr && downstreamInside &&
                    downstreamModule->type() ==
                        rectai::ModuleType::kFilter) {
                    const auto* filterModule =
                        dynamic_cast<const rectai::FilterModule*>(
                            downstreamModule);

                    float filterFreqParam =
                        downstreamModule->GetParameterOrDefault(
                            "freq",
                            downstreamModule->default_parameter_value(
                                "freq"));
                    // Frequency controls are modelled as normalised
                    // parameters in [0,1]. Some .rtp patches may
                    // contain legacy values outside that range; if
                    // used directly, the base+range mapping would
                    // yield extremely high cutoffs (≈186 kHz) that
                    // make the filter effectively transparent. Clamp
                    // the value here so the cutoff stays in the
                    // expected range.
                    filterFreqParam =
                        juce::jlimit(0.0F, 1.0F, filterFreqParam);
                    const double fb =
                        downstreamModule->base_frequency_hz();
                    const double fr =
                        downstreamModule->frequency_range_hz();
                    filterCutoffHz = fb +
                                     fr *
                                         static_cast<double>(
                                             filterFreqParam);

                    const float qParam =
                        downstreamModule->GetParameterOrDefault(
                            "q", downstreamModule->default_parameter_value(
                                      "q"));
                    const float minQ = 0.5F;
                    const float maxQ = 10.0F;
                    filterQ = minQ + (maxQ - minQ) * qParam;
                    // Map FilterModule::Mode to AudioEngine filter
                    // mode.
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
                        // Default to low-pass if we do not know the
                        // mode.
                        filterMode = 1;
                    }
                }

                audioEngine_.setVoiceFilter(assignedVoice, filterMode,
                                            filterCutoffHz, filterQ);
                audioEngine_.setVoiceWaveform(assignedVoice,
                                              waveformIndex);

                if (oscModule != nullptr) {
                    const auto& env = oscModule->envelope();

                    // El motor de audio implementa actualmente una
                    // envolvente simple tipo AR (Attack/Release). Para
                    // que el parámetro `decay` de los Oscillators tenga
                    // un efecto audible incluso cuando `release` es 0 en
                    // el `.rtp`, mapeamos el tiempo de release efectivo
                    // como `env.release` si es > 0, o en su defecto como
                    // `env.decay`.
                    const float effectiveReleaseMs =
                        (env.release > 0.0F) ? env.release : env.decay;

                    audioEngine_.setVoiceEnvelope(
                        assignedVoice, env.attack, env.decay,
                        env.duration, effectiveReleaseMs);
                }

                // Mark generator and, when present, its downstream
                // module as visually active so the paint layer can
                // render waveforms on those paths. For visuals
                // consider only mutes at source or on the direct
                // connection; a mute on the downstream module's radial
                // (e.g. Filter→Master) should still allow Osc→Filter
                // to display a waveform even though the chain is
                // silent at the master.
                const bool visualChainMuted =
                    srcMuted || route.connectionMuted;
                if (!visualChainMuted) {
                    modulesWithActiveAudio_.insert(module->id());
                    moduleVoiceIndex_[module->id()] = assignedVoice;
                    if (downstreamModule != nullptr && downstreamInside &&
                        !route.connectionMuted) {
                        modulesWithActiveAudio_.insert(
                            downstreamModule->id());
                        moduleVoiceIndex_[downstreamModule->id()] =
                            assignedVoice;
                    }
                }
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
    // Synchronise the per-connection visual source map with the
    // current Scene, modules and module→voice mapping. This allows
    // the paint layer to query a single abstraction per connection
    // (pre/post voice or Sampleplay) instead of reimplementing
    // routing heuristics based on module types.
    updateConnectionVisualSources();

    // Configure per-connection waveform taps in the audio engine
    // based on the current visual source mapping. Each audio
    // connection is mapped to a low-level source (voice pre/post
    // filter or Sampleplay) so that the engine can maintain an
    // independent waveform history per connection.
    audioEngine_.clearAllConnectionWaveformTaps();

    // Use the engine-owned AudioGraph as the canonical list of audio
    // edges that can have waveform taps. We still rely on the
    // UI-side ConnectionVisualSource map to decide whether each edge
    // should observe pre/post voice or Sampleplay, but the set of
    // candidate edges comes from AudioGraph::audio_edges().
    using VSKind = MainComponent::ConnectionVisualSource::Kind;

    for (const auto& edge : audioEdges) {
        rectai::Connection tmpConn{edge.from_module_id,
                                   edge.from_port_name,
                                   edge.to_module_id,
                                   edge.to_port_name,
                                   edge.is_hardlink};
        const std::string key = makeConnectionKey(tmpConn);

        const auto vsIt = connectionVisualSources_.find(key);
        if (vsIt == connectionVisualSources_.end()) {
            continue;
        }

        const auto& source = vsIt->second;

        if (source.kind == VSKind::kSampleplay) {
            audioEngine_.configureConnectionWaveformTap(
                key,
                AudioEngine::ConnectionTapSourceKind::kSampleplay,
                -1);
        } else if (source.kind == VSKind::kVoicePre ||
                   source.kind == VSKind::kVoicePost) {
            if (source.voiceIndex < 0 ||
                source.voiceIndex >= AudioEngine::kMaxVoices) {
                continue;
            }

            const auto tapKind =
                (source.kind == VSKind::kVoicePre)
                    ? AudioEngine::ConnectionTapSourceKind::kVoicePre
                    : AudioEngine::ConnectionTapSourceKind::kVoicePost;

            audioEngine_.configureConnectionWaveformTap(
                key, tapKind, source.voiceIndex);
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

    // Helper to run one audio step for all Sequencer modules for a
    // given step index, generating one MidiNoteEvent per active step
    // and letting Sampleplay/Oscillator modules consume that event as
    // a shared logical input.
    auto runSequencerStep = [&](const int stepIndex) {
        const auto& modulesLocal = scene_.modules();
        const auto& objectsLocal = scene_.objects();

        // Precompute mapping from module id to object id so we can
        // test spatial predicates and mute state for destinations.
        std::unordered_map<std::string, std::int64_t>
            moduleToObjectIdLocal;
        moduleToObjectIdLocal.reserve(objectsLocal.size());
        for (const auto& [objId, obj] : objectsLocal) {
            moduleToObjectIdLocal.emplace(obj.logical_id(), objId);
        }

        if (stepIndex < 0 ||
            stepIndex >= rectai::SequencerPreset::kNumSteps) {
            return;
        }

        for (const auto& [id, modulePtr] : modulesLocal) {
            if (modulePtr == nullptr) {
                continue;
            }

            if(!modulePtr->is<rectai::SequencerModule>()) {
                continue;
            }

            auto* seqModule = dynamic_cast<rectai::SequencerModule*>(modulePtr.get());
            if (seqModule == nullptr) {
                continue;
            }

            // Require the Sequencer tangible to be present and
            // inside the musical area.
            const auto objIdIt = moduleToObjectIdLocal.find(id);
            if (objIdIt == moduleToObjectIdLocal.end()) {
                continue;
            }

            const auto objIt = objectsLocal.find(objIdIt->second);
            if (objIt == objectsLocal.end()) {
                continue;
            }

            const auto& obj = objIt->second;
            if (!isInsideMusicArea(obj)) {
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
            const int seqVersion = seqModule->version();

            // Si el paso está desactivado, no disparamos notas y,
            // opcionalmente, forzamos silencio explícito en los
            // osciladores conectados solo cuando el Sequencer está
            // autorizado a controlar volumen.
            if (!step.enabled) {
                if (sequencerControlsVolume_) {
                    // When the step is disabled and the Sequencer
                    // is allowed to control volume, explicitly mute
                    // any Oscillator modules downstream of this
                    // Sequencer according to the current graph
                    // topology.
                    for (const auto& edge : graphEdges) {
                        if (edge.from_module_id != id) {
                            continue;
                        }

                        const auto modDestIt =
                            modulesLocal.find(edge.to_module_id);
                        if (modDestIt == modulesLocal.end() ||
                            modDestIt->second == nullptr) {
                            continue;
                        }

                        if (auto* oscModule =
                                dynamic_cast<rectai::OscillatorModule*>(
                                    modDestIt->second.get())) {
                            // Do not alter the user-visible `gain`
                            // parameter (white handle). Instead,
                            // drive a separate Sequencer-controlled
                            // gain factor used for audio and the
                            // grey handle on the volume bar.
                            oscillatorSequencerGain_[oscModule->id()] =
                                0.0F;
                        }
                    }
                }
                continue;
            }

            // For each Sequencer, walk outgoing connections and
            // drive downstream modules that can consume what the
            // Sequencer produces (Sampleplay/Oscillator for ahora).
            for (const auto& edge : graphEdges) {
                if (edge.from_module_id != id) {
                    continue;
                }

                const auto toObjIdIt =
                    moduleToObjectIdLocal.find(edge.to_module_id);
                if (toObjIdIt == moduleToObjectIdLocal.end()) {
                    continue;
                }

                const auto dstObjIt =
                    objectsLocal.find(toObjIdIt->second);
                if (dstObjIt == objectsLocal.end()) {
                    continue;
                }

                const auto& dstObj = dstObjIt->second;

                // Respect musical area and geometric cone for
                // non-hardlink connections.
                if (!isInsideMusicArea(dstObj) ||
                    (!edge.is_hardlink &&
                     !isConnectionGeometricallyActive(obj, dstObj))) {
                    continue;
                }

                const auto modDestIt =
                    modulesLocal.find(edge.to_module_id);
                if (modDestIt == modulesLocal.end() ||
                    modDestIt->second == nullptr) {
                    continue;
                }

                auto* dstModule = modDestIt->second.get();

                // Connection-level mute state for this Sequencer →
                // destination edge.
                rectai::Connection tmpConn{
                    edge.from_module_id,
                    edge.from_port_name,
                    edge.to_module_id,
                    edge.to_port_name,
                    edge.is_hardlink};
                const std::string connKey =
                    makeConnectionKey(tmpConn);
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
                noteEvent.channel = 0;  // Single logical channel for now.
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
                    // Respect the destination module's route to the
                    // master: if all Sampleplay → Output
                    // (MASTER_OUTPUT_ID)
                    // connections are muted, do not trigger notes
                    // for this step.
                    bool dstMutedToMaster = false;
                    {
                        bool hasMasterRoute = false;
                        bool masterRouteMuted = true;

                        for (const auto& aedge : audioEdges) {
                            if (aedge.from_module_id !=
                                    sampleModule->id() ||
                                aedge.to_module_id != rectai::MASTER_OUTPUT_ID) {
                                continue;
                            }

                            hasMasterRoute = true;

                            rectai::Connection mconn{
                                aedge.from_module_id,
                                aedge.from_port_name,
                                aedge.to_module_id,
                                aedge.to_port_name,
                                aedge.is_hardlink};

                            const std::string mkey =
                                makeConnectionKey(mconn);
                            const bool connIsMuted =
                                mutedConnections_.find(mkey) !=
                                mutedConnections_.end();
                            if (!connIsMuted) {
                                masterRouteMuted = false;
                                break;
                            }
                        }

                        dstMutedToMaster = hasMasterRoute && masterRouteMuted;
                    }

                    if (dstMutedToMaster) {
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

                    int midiKey = noteEvent.note;
                    // In Sequencer v1 (pulse mode) we do not send
                    // melodic information: the audible note comes
                    // from the destination module configuration
                    // (`midifreq` in Sampleplay). For v2+ the
                    // sequencer can override pitch using
                    // `SequencerStep::pitch`.
                    if (seqVersion == 1) {
                        const float midiNote =
                            sampleModule->GetParameterOrDefault(
                                "midifreq", 87.0F);
                        midiKey = juce::jlimit(
                            0, 127,
                            static_cast<int>(std::floor(
                                static_cast<double>(midiNote))));
                    }

                    audioEngine_.triggerSampleplayNote(
                        activeInst->bank, activeInst->program,
                        midiKey, effectiveVelocity);

                    modulesWithActiveAudio_.insert(
                        sampleModule->id());
                    continue;
                }

                // 2) Oscillator: update freq/gain based on the
                // MidiNoteEvent; the audio engine will pick up these
                // changes on the next tick. El tiempo de nota audible
                // se gobierna ahora únicamente por la envolvente de la
                // voz (attack/release) y por los pasos activos del
                // Sequencer; no se aplica un gating adicional basado
                // en `speed`.
                if (auto* oscModule = dynamic_cast<rectai::OscillatorModule*>(
                        dstModule)) {
                    // In Sequencer v1 (pulse mode) we keep the
                    // Oscillator's pitch as configured on the
                    // destination module (midifreq/freq). Only in
                    // v2+ the Sequencer is allowed to drive
                    // oscillator frequency from `step.pitch`.
                    if (seqVersion >= 2) {
                        const double targetHz =
                            440.0 *
                            std::pow(2.0,
                                     (static_cast<double>(
                                          noteEvent.note) -
                                      69.0) /
                                         12.0);

                        const double baseHz =
                            oscModule->base_frequency_hz();
                        const double rangeHz =
                            oscModule->frequency_range_hz();
                        if (rangeHz > 0.0) {
                            const double norm =
                                (targetHz - baseHz) / rangeHz;
                            const float freqParam = juce::jlimit(
                                0.0F, 1.0F,
                                static_cast<float>(norm));
                            scene_.SetModuleParameter(
                                oscModule->id(), "freq", freqParam);
                        }
                    }

                    if (sequencerControlsVolume_) {
                        const float gainFactor = juce::jlimit(
                            0.0F, 1.0F, noteEvent.velocity01);
                        // Store the Sequencer-driven gain separately
                        // from the Oscillator's own `gain` parameter
                        // so that the white handle continues to
                        // represent the module output level while
                        // the grey handle (and audio) follow this
                        // factor, clamped to not exceed the white
                        // value.
                        oscillatorSequencerGain_[oscModule->id()] =
                            gainFactor;
                    }

                    // Retrigger the per-voice amplitude envelope for
                    // the Oscillator mapped to this module so that
                    // each active Sequencer step produces a fresh
                    // ADSR-style pulse, even when the underlying
                    // voice level does not toggle between 0 y >0.
                    const auto voiceIt =
                        moduleVoiceIndex_.find(oscModule->id());
                    if (voiceIt != moduleVoiceIndex_.end()) {
                        const int voiceIndex = voiceIt->second;
                        if (voiceIndex >= 0 &&
                            voiceIndex < AudioEngine::kMaxVoices) {
                            audioEngine_.triggerVoiceEnvelope(
                                voiceIndex);
                        }
                    }
                }
            }
        }
    };

    // Derive the current global transport position in beats from the
    // audio engine so that Sequencer timing and visual beat pulses
    // stay locked to the audio callback instead of the GUI timer.
    const double engineBeats = audioEngine_.transportBeats();
    double wholeBeatsDouble = 0.0;
    const double fracBeat = std::modf(engineBeats, &wholeBeatsDouble);
    const int wholeBeats = static_cast<int>(wholeBeatsDouble);

    // Generate BPM pulses and Sampleplay beat triggers for each new
    // whole beat that elapsed since the last timer tick.
    const int prevWholeBeats = static_cast<int>(transportBeats_);
    if (wholeBeats > prevWholeBeats) {
        for (int b = prevWholeBeats; b < wholeBeats; ++b) {
            const int beatInBar = b % 4;
            const bool strong = (beatInBar == 0);
            pulses_.push_back(Pulse{0.0F, strong});
            triggerSampleplayNotesOnBeat(strong);
        }
    }

    transportBeats_ = static_cast<double>(wholeBeats);
    beatPhase_ = juce::jlimit(0.0, 1.0, fracBeat);
    beatIndex_ = wholeBeats % 4;

    // Advance Sequencer audio step at 16 steps per bar (sixteenth
    // notes) so that, for a 4/4 meter, a full 16-step pattern is
    // traversed over 4 beats. The step index is now derived from the
    // same global transport position in beats (integer beats
    // `transportBeats_` + fractional `beatPhase_`) that drives Loop
    // playback and the central BPM pulses, ensuring all components
    // stay phase-locked.
    const int audioStepsPerBar = rectai::SequencerPreset::kNumSteps;
    if (audioStepsPerBar > 0) {
        // Derive the current step **counter** from the global transport
        // position (integer beats + fractional phase) using a fixed
        // beats-per-step. Esto nos da un índice absoluto de step
        // desde el inicio del transporte, no sólo el valor módulo
        // 16; así, si el timer llega tarde y se han cruzado varios
        // steps ideales entre dos callbacks, podemos ejecutar todos
        // los pasos intermedios en orden y evitar saltos rítmicos.
        const double transportPositionBeats = transportBeats_ + beatPhase_;
        const double beatsPerStep = 1.0 / 4.0;  // 16 steps over 4 beats.
        const double stepPhase =
            (beatsPerStep > 0.0)
                ? (transportPositionBeats / beatsPerStep)
                : 0.0;

        const auto newStepCounter = static_cast<std::int64_t>(
            std::floor(stepPhase));

        if (newStepCounter > sequencerAudioStepCounter_) {
            // Avanzar uno a uno todos los steps que hayan quedado
            // pendientes desde el último tick del timer. Cada
            // incremento del contador corresponde a un step lógico
            // (semicorchea); el índice visible del preset se obtiene
            // módulo `audioStepsPerBar`.
            for (std::int64_t s = sequencerAudioStepCounter_ + 1;
                 s <= newStepCounter; ++s) {
                const int stepIndex = static_cast<int>(
                    s % static_cast<std::int64_t>(audioStepsPerBar));
                sequencerAudioStep_ = stepIndex;
                runSequencerStep(stepIndex);
            }

            sequencerAudioStepCounter_ = newStepCounter;
        } else if (newStepCounter < sequencerAudioStepCounter_) {
            // El transporte se ha reseteado o ha retrocedido
            // (por ejemplo, al reinicializar el dispositivo de
            // audio). Sin intentar reproducir steps "en
            // retroceso", re-sincronizamos el contador al valor
            // actual y ejecutamos el step correspondiente una sola
            // vez para alinear el runtime con la nueva posición.
            sequencerAudioStepCounter_ = newStepCounter;
            const int stepIndex = static_cast<int>(
                (newStepCounter % audioStepsPerBar + audioStepsPerBar) %
                audioStepsPerBar);
            sequencerAudioStep_ = stepIndex;
            runSequencerStep(stepIndex);
        }
    }

    // Advance connection flow phase (used for pulses along edges).
    connectionFlowPhase_ += dt;
    if (connectionFlowPhase_ > 1.0) {
        connectionFlowPhase_ -= std::floor(connectionFlowPhase_);
    }

    // Simple sequencer phase for widgets (steps per bar = 8).
    const int stepsPerBar = 8;
    const double bps = bpm_ / 60.0;  // beats per second
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
