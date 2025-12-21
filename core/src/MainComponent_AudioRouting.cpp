#include "MainComponent.h"

#include <limits>

#include "AudioEngine.h"
#include "core/AudioModules.h"

void MainComponent::updateAudioRoutingAndVoices(
    const std::unordered_map<std::int64_t, rectai::ObjectInstance>& objects,
    const std::unordered_map<std::string, std::unique_ptr<rectai::AudioModule>>& modules,
    const std::vector<rectai::AudioGraph::Edge>& audioEdges,
    const std::unordered_map<std::string, std::int64_t>& moduleToObjectId,
    const float globalVolumeGain)
{
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
            juce::ignoreUnused(objId);

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
            juce::ignoreUnused(objId);

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

            const auto mode = bestFilter->current_mode();
            sampleplayFilterMode = mode->id;
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

        const auto* oscModule =
            dynamic_cast<const rectai::OscillatorModule*>(module);

        const float freqParam = module->GetParameterOrDefault(
            "freq", module->default_parameter_value("freq"));
        const double frequency =
            module->base_frequency_hz() +
            module->frequency_range_hz() * static_cast<double>(freqParam);

        const float baseLevel = module->base_level();
        const float levelRange = module->level_range();

        int waveformIndex = module->current_mode_index();

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
                        const auto& mode = filterModule->current_mode();
                        filterMode = mode->id;
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

                    const EnvelopeAnalysis analysis =
                        analyseEnvelope(env);

                    // For backwards compatibility with existing
                    // patches, preserve the previous behaviour of
                    // `release`: when the .rtp defines a
                    // release>0, use it directly; otherwise fall
                    // back to `decay` as the effective release.
                    const float effectiveReleaseMs =
                        (env.release > 0.0F) ? env.release : env.decay;

                    // When a clear sustain plateau is detected,
                    // treat the envelope as an ADSR gated by the
                    // lifetime of the Oscillator (duration=0).
                    // Otherwise, honour `env.duration` to keep the
                    // original one-shot behaviour (as in the default
                    // Oscillator saw in default.rtp).
                    const float durationMsToUse =
                        analysis.hasSustainPlateau ? 0.0F
                                                    : env.duration;

                    audioEngine_.setVoiceEnvelope(
                        assignedVoice, env.attack, env.decay,
                        durationMsToUse, effectiveReleaseMs,
                        analysis.sustainLevel);
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
}
