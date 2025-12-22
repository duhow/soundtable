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
    static constexpr std::array<double, 6> kBeats = {
        0.125, // 1/32 note
        0.25,  // 1/16 note
        0.5,   // 1/8 note
        1.0,   // 1/4 note
        2.0,   // 2/4 note
        4.0    // 4/4 note
    };

    const int idx = std::min(5, std::max(0, speed));
    return kBeats[static_cast<size_t>(idx)];
}

// Shared analysis of Reactable-style envelopes to derive a sustain
// level and whether a clear sustain plateau exists. This is used
// both when initialising per-voice envelopes and when updating
// sustain in place after UI edits.
struct EnvelopeAnalysis {
    float sustainLevel{1.0F};
    bool hasSustainPlateau{false};
};

[[nodiscard]] EnvelopeAnalysis analyseEnvelope(const rectai::Envelope& e)
{
    EnvelopeAnalysis result{};

    const auto& xs = e.points_x;
    const auto& ys = e.points_y;
    const std::size_t n = ys.size();
    if (n == 0 || xs.size() != n) {
        result.sustainLevel = 1.0F;
        result.hasSustainPlateau = false;
        return result;
    }

    // Buscamos el tramo "plano" interno más largo con y>0 y
    // anchura mínima en X. Ese tramo se interpreta como sustain.
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

        const float width = static_cast<float>(xs[j - 1] - xs[i]);
        if (width > bestWidth) {
            bestWidth = width;
            bestLevel = y;
        }

        i = j;
    }

    if (bestWidth >= kMinPlateauWidth && bestLevel > 0.0F) {
        result.sustainLevel = bestLevel;
        result.hasSustainPlateau = true;
        return result;
    }

    // Sin plateau claro: usamos como referencia el máximo y>0 previo
    // al último punto, pero lo marcaremos como envelope one-shot.
    float maxY = 0.0F;
    for (std::size_t i = 0; i + 1 < n; ++i) {
        maxY = std::max(maxY, ys[i]);
    }
    if (maxY > 0.0F) {
        result.sustainLevel = maxY;
    } else {
        result.sustainLevel = 1.0F;
    }

    result.hasSustainPlateau = false;
    return result;
}

}  // namespace

void MainComponent::refreshInsideMusicAreaFlags()
{
    // Take a snapshot so we can upsert safely while iterating.
    const auto objectsSnapshot = scene_.objects();
    for (const auto& [id, obj] : objectsSnapshot) {
        juce::ignoreUnused(id);

        // Recompute the inside-music flag in table space. Avoid
        // touching the Scene when the flag has not changed since the
        // previous tick to keep UpsertObject traffic minimal.
        const bool inside = computeInsideMusicArea(obj);
        if (inside == obj.inside_music_area()) {
            continue;
        }

        auto updated = obj;
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

void MainComponent::updateRotationDrivenControllers(
    const std::unordered_map<std::int64_t, rectai::ObjectInstance>& objects,
    const std::unordered_map<std::string, std::unique_ptr<rectai::AudioModule>>& modules,
    const std::unordered_map<std::int64_t, float>& rotationDeltaDegrees)
{
    // ------------------------------------------------------------------
    // Rotation gestures → module parameters.
    // For each tangible object with a rotation delta we:
    //   - adjust `freq` on any AudioModule that uses frequency control
    //     (typical generators/filters), except when the object is docked.
    //   - adjust per-module pitch for Oscillators/Sampleplay.
    //   - select Loop samples.
    //   - adjust global tempo/BPM when the module is a TempoModule.
    //   - adjust global master volume when the module is a VolumeModule.
    // All mappings share the same rotation sense: clockwise increases
    // the controlled value, counter-clockwise decreases it.
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

        // 1) Rotation → per-module frequency (`freq` parameter) or
        // Loop sample selection (`sample` parameter).
        if (auto* loopModule =
                dynamic_cast<rectai::LoopModule*>(module)) {
            if (!obj.docked()) {
                const float deltaSample = -diff / 360.0F;  // [-0.5, 0.5]

                if (std::fabs(deltaSample) >
                    std::numeric_limits<float>::epsilon()) {
                    const float currentSampleParam =
                        loopModule->GetParameterOrDefault("sample", 0.0F);
                    float clampedCurrent = juce::jlimit(
                        0.0F, 1.0F, currentSampleParam);
                    float updated = clampedCurrent + deltaSample;

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
        } else if (module->uses_pitch_control() && !obj.docked()) {
            const float deltaSemitones =
                -diff * (12.0F / 360.0F);  // approx [-6, 6]

            if (std::fabs(deltaSemitones) <=
                std::numeric_limits<float>::epsilon()) {
                continue;
            }

            constexpr float kMinMidi = 24.0F;
            constexpr float kMaxMidi = 108.0F;

            const float currentMidi = module->GetParameterOrDefault(
                "midifreq", 57.0F);
            const float clampedCurrent = juce::jlimit(
                kMinMidi, kMaxMidi, currentMidi);
            float updatedMidi = clampedCurrent + deltaSemitones;
            updatedMidi = juce::jlimit(kMinMidi, kMaxMidi, updatedMidi);

            auto* oscModule =
                dynamic_cast<rectai::OscillatorModule*>(module);
            float effectivePrevMidi = clampedCurrent;
            float effectiveNewMidi = updatedMidi;
            if (oscModule != nullptr &&
                oscModule->play_midi_note_from_rotation()) {
                effectivePrevMidi = std::round(clampedCurrent);
                effectiveNewMidi = std::round(updatedMidi);
                effectivePrevMidi = juce::jlimit(kMinMidi, kMaxMidi,
                                                 effectivePrevMidi);
                effectiveNewMidi = juce::jlimit(kMinMidi, kMaxMidi,
                                                 effectiveNewMidi);
            }

            scene_.SetModuleParameter(obj.logical_id(), "midifreq",
                                      updatedMidi);

            if (oscModule != nullptr) {
                const double targetHz = 440.0 *
                                        std::pow(
                                            2.0,
                                            (static_cast<double>(
                                                 (oscModule->
                                                      play_midi_note_from_rotation()
                                                       ? effectiveNewMidi
                                                       : updatedMidi)) -
                                             69.0) /
                                                12.0);

                const double baseHz = oscModule->base_frequency_hz();
                const double rangeHz = oscModule->frequency_range_hz();
                if (rangeHz > 0.0) {
                    const double norm = (targetHz - baseHz) / rangeHz;
                    const float freqParam = juce::jlimit(
                        0.0F, 1.0F,
                        static_cast<float>(norm));
                    scene_.SetModuleParameter(obj.logical_id(), "freq",
                                              freqParam);

                    const float comparePrev =
                        oscModule->play_midi_note_from_rotation()
                            ? effectivePrevMidi
                            : clampedCurrent;
                    const float compareNew =
                        oscModule->play_midi_note_from_rotation()
                            ? effectiveNewMidi
                            : updatedMidi;

                    if (std::abs(compareNew - comparePrev) >
                        1.0e-4F) {
                        maybeRetriggerOscillatorOnFreqChange(
                            obj.logical_id(), module);
                    }
                }
            }
        } else if (module->uses_frequency_control() && !obj.docked()) {
            const float deltaFreq = -diff / 360.0F;  // [-0.5, 0.5]

            if (std::fabs(deltaFreq) <=
                std::numeric_limits<float>::epsilon()) {
                continue;
            }

            const float currentFreq = module->GetParameterOrDefault(
                "freq", module->default_parameter_value("freq"));
            const float newFreq = juce::jlimit(
                0.0F, 1.0F, currentFreq + deltaFreq);

            if (std::abs(newFreq - currentFreq) <= 1.0e-4F) {
                continue;
            }

            scene_.SetModuleParameter(obj.logical_id(), "freq",
                                      newFreq);

            maybeRetriggerOscillatorOnFreqChange(obj.logical_id(),
                                                 module);
        }

        // 2) Rotation → global tempo (BPM).
        if (auto* tempoModule =
                dynamic_cast<rectai::TempoModule*>(module)) {
            (void)tempoModule;

            if (std::fabs(diff) <=
                std::numeric_limits<float>::epsilon()) {
                continue;
            }

            const float deltaBpm = -diff / 5.0F;
            if (std::fabs(deltaBpm) <=
                std::numeric_limits<float>::epsilon()) {
                continue;
            }

            const float newBpm =
                rectai::TempoModule::ClampBpm(bpm_ + deltaBpm);

            bpm_ = newBpm;

            bpmLastChangeSeconds_ =
                juce::Time::getMillisecondCounterHiRes() / 1000.0;

            scene_.SetModuleParameter(module->id(), "tempo", bpm_);
        }

        // 3) Rotation → global volume (master).
        if (auto* volumeModule =
                dynamic_cast<rectai::VolumeModule*>(module)) {
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
}

void MainComponent::runSequencerStep(
    const int stepIndex,
    const std::vector<rectai::AudioGraph::Edge>& graphEdges,
    const std::vector<rectai::AudioGraph::Edge>& audioEdges,
    const float globalVolumeGain)
{
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

        if (!modulePtr->is<rectai::SequencerModule>()) {
            continue;
        }

        auto* seqModule =
            dynamic_cast<rectai::SequencerModule*>(modulePtr.get());
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
                        oscillatorSequencerGain_[oscModule->id()] =
                            0.0F;
                    }
                }
            }
            continue;
        }

        // For each Sequencer, walk outgoing connections and drive
        // downstream modules that can consume what the Sequencer
        // produces (Sampleplay/Oscillator for ahora).
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

            if (connectionMuted) {
                continue;
            }

            rectai::MidiNoteEvent noteEvent;
            noteEvent.channel = 0;
            noteEvent.note = juce::jlimit(0, 127, step.pitch);
            noteEvent.velocity01 =
                juce::jlimit(0.0F, 1.0F, step.velocity01);
            noteEvent.timeBeats = transportBeats_;
            noteEvent.is_note_on = true;

            if (const auto* sampleModule =
                    dynamic_cast<const rectai::SampleplayModule*>(
                        dstModule)) {
                bool dstMutedToMaster = false;
                {
                    bool hasMasterRoute = false;
                    bool masterRouteMuted = true;

                    for (const auto& aedge : audioEdges) {
                        if (aedge.from_module_id !=
                                sampleModule->id() ||
                            aedge.to_module_id !=
                                rectai::MASTER_OUTPUT_ID) {
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

                    dstMutedToMaster =
                        hasMasterRoute && masterRouteMuted;
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

                const float stepVelocity =
                    sequencerControlsVolume_
                        ? noteEvent.velocity01
                        : 1.0F;

                float effectiveVelocity =
                    chainLevel * globalVolumeGain *
                    stepVelocity;
                if (effectiveVelocity <= 0.0F) {
                    continue;
                }

                effectiveVelocity = juce::jlimit(0.0F, 1.0F,
                                                 effectiveVelocity);

                int midiKey = noteEvent.note;
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

            if (auto* oscModule =
                    dynamic_cast<rectai::OscillatorModule*>(
                        dstModule)) {
                if (seqVersion >= 2) {
                    const double targetHz =
                        440.0 *
                        std::pow(2.0,
                                 (static_cast<double>(noteEvent.note) -
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
                    oscillatorSequencerGain_[oscModule->id()] =
                        gainFactor;
                }

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
}

void MainComponent::updateSequencerAndPulses(
    const std::vector<rectai::AudioGraph::Edge>& graphEdges,
    const std::vector<rectai::AudioGraph::Edge>& audioEdges,
    const float globalVolumeGain,
    const double nowSeconds,
    const double dt)
{
    // Age existing pulses and remove the ones that have fully faded.
    constexpr double pulseLifetimeSeconds = 0.35;
    const float ageStep =
        static_cast<float>(dt / pulseLifetimeSeconds);

    for (auto& pulse : pulses_) {
        pulse.age += ageStep;
    }

    pulses_.erase(std::remove_if(pulses_.begin(), pulses_.end(),
                                 [](const Pulse& p) {
                                     return p.age >= 1.0F;
                                 }),
                  pulses_.end());

    // Derive the current global transport position in beats from the
    // audio engine so that Sequencer timing and visual beat pulses
    // stay locked to the audio callback instead of the GUI timer.
    const double engineBeats = audioEngine_.transportBeats();
    double wholeBeatsDouble = 0.0;
    const double fracBeat =
        std::modf(engineBeats, &wholeBeatsDouble);
    const int wholeBeats = static_cast<int>(wholeBeatsDouble);

    constexpr double kPulsesPerBeat = 2.0;
    constexpr int kPulsesPerBar = 8;
    constexpr double kPulsePhaseOffsetBeats = -1.0;

    const double pulsePositionBeats =
        engineBeats + kPulsePhaseOffsetBeats;
    int pulseStepNow = static_cast<int>(std::floor(
        pulsePositionBeats * kPulsesPerBeat));

    if (pulseStepNow < lastPulseStep_) {
        lastPulseStep_ = pulseStepNow;
    }

    if (pulseStepNow > lastPulseStep_) {
        for (int s = lastPulseStep_ + 1; s <= pulseStepNow; ++s) {
            const int stepInBar =
                (s % kPulsesPerBar + kPulsesPerBar) %
                kPulsesPerBar;

            const bool isQuarterStep = (stepInBar % 2) == 0;
            const bool strong = (stepInBar == 6);

            if (isQuarterStep) {
                pulses_.push_back(Pulse{0.0F, strong});
                triggerSampleplayNotesOnBeat(strong);
            }
        }
        lastPulseStep_ = pulseStepNow;
    }

    transportBeats_ = static_cast<double>(wholeBeats);
    beatPhase_ = juce::jlimit(0.0, 1.0, fracBeat);
    beatIndex_ = wholeBeats % 4;

    const int audioStepsPerBar =
        rectai::SequencerPreset::kNumSteps;
    if (audioStepsPerBar > 0) {
        const double transportPositionBeats =
            transportBeats_ + beatPhase_;
        const double beatsPerStep = 1.0 / 4.0;
        const double stepPhase =
            (beatsPerStep > 0.0)
                ? (transportPositionBeats / beatsPerStep)
                : 0.0;

        const auto newStepCounter =
            static_cast<std::int64_t>(
                std::floor(stepPhase));

        if (newStepCounter > sequencerAudioStepCounter_) {
            for (std::int64_t s = sequencerAudioStepCounter_ + 1;
                 s <= newStepCounter; ++s) {
                const int stepIndex = static_cast<int>(
                    s % static_cast<std::int64_t>(
                            audioStepsPerBar));
                sequencerAudioStep_ = stepIndex;
                runSequencerStep(stepIndex, graphEdges,
                                 audioEdges, globalVolumeGain);
            }

            sequencerAudioStepCounter_ = newStepCounter;
        } else if (newStepCounter <
                   sequencerAudioStepCounter_) {
            sequencerAudioStepCounter_ = newStepCounter;
            const int stepIndex = static_cast<int>(
                (newStepCounter % audioStepsPerBar +
                 audioStepsPerBar) %
                audioStepsPerBar);
            sequencerAudioStep_ = stepIndex;
            runSequencerStep(stepIndex, graphEdges, audioEdges,
                             globalVolumeGain);
        }
    }

    // Advance connection flow phase (used for pulses along edges).
    connectionFlowPhase_ += dt;
    if (connectionFlowPhase_ > 1.0) {
        connectionFlowPhase_ -=
            std::floor(connectionFlowPhase_);
    }

    // Simple sequencer phase for widgets (steps per bar = 8).
    const int stepsPerBar = 8;
    const double bps = bpm_ / 60.0;
    const double stepsPerSecond =
        bps * static_cast<double>(stepsPerBar);
    sequencerPhase_ += stepsPerSecond * dt;
    if (sequencerPhase_ >= 1.0) {
        sequencerPhase_ -= 1.0;
    }
    const int newStep = static_cast<int>(
        sequencerPhase_ * static_cast<double>(stepsPerBar));
    if (newStep != sequencerStep_) {
        sequencerStep_ = newStep;
    }
}

MainComponent::GeometryCache MainComponent::buildGeometryCache() const
{
    GeometryCache cache;
    const auto& objects = scene_.objects();
    cache.moduleToObjectId.reserve(objects.size());

    for (const auto& [objId, obj] : objects) {
        cache.moduleToObjectId.emplace(obj.logical_id(), objId);
    }

    return cache;
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

    updateRotationDrivenControllers(objects, modules, rotationDeltaDegrees);

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

        // Objects currently inside the musical area along with their
        // screen-space positions so that both module↔module and
        // module↔centre collisions can reuse the same mapping.
        struct InsideEntry {
            std::int64_t id;
            const rectai::ObjectInstance* obj{nullptr};
            float sx{0.0F};
            float sy{0.0F};
        };

        std::vector<InsideEntry> inside;
        inside.reserve(objects.size());
        for (const auto& [objId, obj] : objects) {
            if (!isInsideMusicArea(obj)) {
                continue;
            }

            const auto pos = objectTableToScreen(obj, bounds);
            InsideEntry entry;
            entry.id = objId;
            entry.obj = &obj;
            entry.sx = pos.x;
            entry.sy = pos.y;
            inside.push_back(entry);
        }

        // Detect new collisions between objects (touching circles) and
        // toggle hardlinks on each new contact event. Use the same
        // centre-origin table mapping as the renderer so that the
        // collision radius matches the visual node circles.
        std::unordered_set<std::string> currentPairs;
        for (std::size_t i = 0; i < inside.size(); ++i) {
            const auto& entryA = inside[i];
            const auto* objA = entryA.obj;

            // Ignore collisions involving the invisible Output/master
            // module (id MASTER_OUTPUT_ID). Its logical route is always
            // represented by the radial line from the object to the
            // centre of the table and is managed via implicit
            // module→MASTER_OUTPUT_ID connections plus per-connection
            // mute state, not by creating/removing hardlinks through
            // physical collisions.
            if (objA->logical_id() == rectai::MASTER_OUTPUT_ID) {
                continue;
            }

            const float ax = entryA.sx;
            const float ay = entryA.sy;

            for (std::size_t j = i + 1; j < inside.size(); ++j) {
                const auto& entryB = inside[j];
                const auto* objB = entryB.obj;

                if (objB->logical_id() == rectai::MASTER_OUTPUT_ID) {
                    continue;
                }

                const float bx = entryB.sx;
                const float by = entryB.sy;

                const float dx = bx - ax;
                const float dy = by - ay;
                const float distSq = dx * dx + dy * dy;
                if (distSq > maxDistSq) {
                    continue;
                }

                const std::string pairKey =
                    makeObjectPairKey(entryA.id, entryB.id);
                currentPairs.insert(pairKey);

                if (activeHardlinkCollisions_.find(pairKey) ==
                    activeHardlinkCollisions_.end()) {
                    // On first evaluation after loading a session we
                    // only seed the collision set so that existing
                    // hardlinks are preserved; subsequent re-contacts
                    // after separation will toggle as usual.
                    if (!hardlinkCollisionsInitialised_) {
                        activeHardlinkCollisions_.insert(pairKey);
                    } else {
                        toggleHardlinkBetweenObjects(entryA.id, entryB.id);
                        activeHardlinkCollisions_.insert(pairKey);
                    }
                }
            }
        }

        // Detect collisions between modules and the centre of the table
        // to toggle hardlinks between those modules and the invisible
        // Output/master module (id MASTER_OUTPUT_ID). From the user's
        // perspective this corresponds to "bumping" a tangible against
        // the central node to promote/demote its route to the master.
        std::optional<std::int64_t> masterObjectId;
        for (const auto& [objId, obj] : objects) {
            if (obj.logical_id() == rectai::MASTER_OUTPUT_ID) {
                masterObjectId = objId;
                break;
            }
        }

        if (masterObjectId.has_value()) {
            const float centreX = bounds.getCentreX();
            const float centreY = bounds.getCentreY();

            for (const auto& entry : inside) {
                if (entry.obj == nullptr ||
                    entry.obj->logical_id() == rectai::MASTER_OUTPUT_ID) {
                    continue;
                }

                const float dx = entry.sx - centreX;
                const float dy = entry.sy - centreY;
                const float distSq = dx * dx + dy * dy;
                if (distSq > maxDistSq) {
                    continue;
                }

                const std::string pairKey = makeObjectPairKey(
                    entry.id, *masterObjectId);
                currentPairs.insert(pairKey);

                if (activeHardlinkCollisions_.find(pairKey) ==
                    activeHardlinkCollisions_.end()) {
                    if (!hardlinkCollisionsInitialised_) {
                        activeHardlinkCollisions_.insert(pairKey);
                    } else {
                        toggleHardlinkBetweenObjects(entry.id,
                                                     *masterObjectId);
                        activeHardlinkCollisions_.insert(pairKey);
                    }
                }
            }
        }

        // Clear pairs that are no longer colliding so that a future
        // contact between them (including module↔master centre bumps)
        // can toggle the hardlink again.
        for (auto it = activeHardlinkCollisions_.begin();
             it != activeHardlinkCollisions_.end();) {
            if (currentPairs.find(*it) == currentPairs.end()) {
                it = activeHardlinkCollisions_.erase(it);
            } else {
                ++it;
            }
        }

        // Mark collisions as initialised after the first pass so that
        // future contact events can safely toggle hardlinks.
        hardlinkCollisionsInitialised_ = true;

        // Remove hardlink connections whose endpoints are outside the
        // musical area.
        const auto geometry = buildGeometryCache();
        const auto& moduleToObjectId = geometry.moduleToObjectId;

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
                if (fromObjIdIt != moduleToObjectId.end() &&
                    toObjIdIt != moduleToObjectId.end()) {
                    const auto pairKey = makeObjectPairKey(
                        fromObjIdIt->second, toObjIdIt->second);
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
    auto geometry = buildGeometryCache();
    const auto& moduleToObjectId = geometry.moduleToObjectId;

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

    updateAudioRoutingAndVoices(objects, modules, audioEdges,
                                moduleToObjectId, globalVolumeGain);

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

    // Inform the audio engine whether there is any module currently
    // carrying audible audio so that it can avoid running the full
    // synthesis/sampling path when the scene is completely idle (no
    // modules in the musical area or all routes effectively silent).
    // audioEngine_.setProcessingActive(!modulesWithActiveAudio_.empty());
    // -----
    // HACK: keep audio processing always active
    // to avoid sync issues
    audioEngine_.setProcessingActive(true);

    // Update BPM pulse animation and Sequencer state using real dt
    // between timer ticks so visuals remain stable even if the timer
    // frequency changes or the event loop hiccups.
    const double nowSeconds =
        juce::Time::getMillisecondCounterHiRes() / 1000.0;
    double dt = nowSeconds - lastTimerSeconds_;
    if (dt <= 0.0 || dt > 0.5) {
        dt = 1.0 / 120.0;
    }
    lastTimerSeconds_ = nowSeconds;

    updateSequencerAndPulses(graphEdges, audioEdges, globalVolumeGain,
                             nowSeconds, dt);

    // Limit the maximum repaint rate independently from the timer
    // frequency. This keeps waveform visualisations and widgets
    // smooth while avoiding calling into JUCE's paint pipeline more
    // often than necessary.

    // Skip repaints entirely when the scene is visually idle to
    // reduce CPU usage. We only refresh the UI when there is some
    // form of ongoing visual activity (audio, pulses, BPM label or
    // an active hold/mute gesture).
    const bool hasVisualActivity =
        !modulesWithActiveAudio_.empty() || !pulses_.empty() ||
        (bpmLastChangeSeconds_ > 0.0 &&
         nowSeconds - bpmLastChangeSeconds_ <= 6.0) ||
        activeConnectionHold_.has_value() ||
        // Keep the UI refreshing while the OSC/TUIO activity label
        // is visible (up to 60 seconds since the last message) or
        // while a short-lived traffic pulse is active.
        (lastInputActivitySeconds_ > 0.0 &&
         nowSeconds - lastInputActivitySeconds_ <= 60.0) ||
        (inputActivityPulseSeconds_ > 0.0 &&
         nowSeconds - inputActivityPulseSeconds_ <= 0.25);

    if (hasVisualActivity) {
        repaintWithRateLimit();
    }
}
