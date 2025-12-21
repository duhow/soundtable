#include "MainComponent.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "AudioEngine.h"
#include "MainComponentHelpers.h"
#include "MainComponent_ModulePanelEnvelope.h"
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

    // Hot callers (paint, audio, input) rely on a cached flag in
    // ObjectInstance so that we do not need to recompute the
    // geometry for every query. The flag is maintained by
    // refreshInsideMusicAreaFlags() and by input handlers when an
    // object is dragged.
    return obj.inside_music_area();
}

bool MainComponent::computeInsideMusicArea(
    const rectai::ObjectInstance& obj) const
{
    if (obj.docked()) {
        return false;
    }

    // Inside-music check in table space: the musical surface is the
    // disc of radius 1 centred at (0,0). ObjectInstance coordinates
    // are expressed directly in that centre-origin system.
    const float dx = obj.x();
    const float dy = obj.y();
    return (dx * dx + dy * dy) <= 1.0F;
}

// Helper to map an ObjectInstance on the table (centre-origin
// coordinates with radius 1) to component-space pixels.
juce::Point<float> MainComponent::objectTableToScreen(
    const rectai::ObjectInstance& obj,
    const juce::Rectangle<float>& bounds)
{
    const auto centre = bounds.getCentre();
    const float tableRadius =
        0.45F * std::min(bounds.getWidth(), bounds.getHeight());

    const float cx = centre.x + obj.x() * tableRadius;
    const float cy = centre.y + obj.y() * tableRadius;
    return {cx, cy};
}

void MainComponent::paint(juce::Graphics& g)
{
    const auto intBounds = getLocalBounds();
    const auto bounds = intBounds.toFloat();
    const auto centre = bounds.getCentre();

    // Global BPM label visibility: show only shortly after the tempo
    // has been updated, then fade out smoothly.
    const double nowSeconds =
        juce::Time::getMillisecondCounterHiRes() / 1000.0;
    double bpmLabelAlpha = 0.0;
    if (bpmLastChangeSeconds_ > 0.0) {
        const double elapsed = nowSeconds - bpmLastChangeSeconds_;
        if (elapsed >= 0.0) {
            if (elapsed <= 5.0) {
                bpmLabelAlpha = 1.0;
            } else if (elapsed <= 6.0) {
                bpmLabelAlpha = 1.0 - (elapsed - 5.0);
            }
        }
    }

    // Dock area used as a reference for positioning small UI widgets
    // on the right-hand side (for example, the OSC/TUIO traffic
    // indicator label). This is computed independently of whether
    // there are currently any docked objects.
    auto dockBoundsUi = bounds;
    const float dockWidthUi = calculateDockWidth(dockBoundsUi.getWidth());
    const juce::Rectangle<float> dockAreaUi =
        dockBoundsUi.removeFromRight(dockWidthUi);

    // ---------------------------------------------------------------------
    // Background: cached static table geometry (black backdrop,
    // coloured disc and soft outer ring). The heavy gradient and
    // edge-table work is rendered into an off-screen image only
    // when the component bounds or table colour change; here we
    // simply blit that image.
    // ---------------------------------------------------------------------
    renderTableBackgroundIfNeeded(intBounds);
    if (!tableBackgroundCache_.isNull()) {
        g.drawImageAt(tableBackgroundCache_, intBounds.getX(),
                      intBounds.getY());
    }

    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    // Fetch recent per-voice mono snapshots from the audio engine so
    // that audio-carrying lines can display waveforms that better
    // match the module or chain that is actually producing sound.
    constexpr int kWaveformPoints = 512;
    constexpr float kWaveformAmplitudeScale = 15.0F;

    struct AudioFrameState {
        float voiceWaveformsPre[AudioEngine::kMaxVoices]
                       [kWaveformPoints]{};
        float voiceWaveformsPost[AudioEngine::kMaxVoices]
                    [kWaveformPoints]{};
        float voiceNormPre[AudioEngine::kMaxVoices]{};
        float voiceNormPost[AudioEngine::kMaxVoices]{};
        float voiceRmsPost[AudioEngine::kMaxVoices]{};
        int voicePeriodSamples[AudioEngine::kMaxVoices]{};
        const std::unordered_set<std::string>* modulesWithActiveAudio{nullptr};
        const std::unordered_map<std::string, int>* moduleVoiceIndex{nullptr};
        const std::unordered_map<std::string, ConnectionVisualSource>*
            connectionVisualSources{nullptr};
    };

    auto computeAudioFrameState = [](AudioEngine& audioEngine) {
        AudioFrameState state;

        auto estimateWaveformPeriod = [](const float* samples,
                                         const int numSamples) {
            if (samples == nullptr || numSamples <= 1) {
                return numSamples;
            }

            // Ignore very small lags when estimating the period so that
            // slowly varying or low-frequency content does not collapse to
            // a 1–2 sample "period". Using such tiny lags would make the
            // visualisation effectively sample a single point repeatedly,
            // which appears as a straight line that jumps up and down
            // instead of a stable sine/saw shape, especially for lower
            // notes driven by the Sequencer.
            const int minLag = 4;
            const int maxLag = std::max(minLag, numSamples / 2);

            float bestCorr = 0.0F;
            int bestLag = numSamples;

            for (int lag = minLag; lag <= maxLag; ++lag) {
                float corr = 0.0F;
                const int limit = numSamples - lag;
                for (int i = 0; i < limit; ++i) {
                    corr += samples[i] * samples[i + lag];
                }

                if (corr > bestCorr) {
                    bestCorr = corr;
                    bestLag = lag;
                }
            }

            if (bestLag < minLag || bestLag >= numSamples) {
                return numSamples;
            }

            return bestLag;
        };

        for (int v = 0; v < AudioEngine::kMaxVoices; ++v) {
            audioEngine.getVoiceWaveformSnapshot(
                v, state.voiceWaveformsPre[v], kWaveformPoints, 0.05);
            audioEngine.getVoiceFilteredWaveformSnapshot(
                v, state.voiceWaveformsPost[v], kWaveformPoints, 0.05);

            float maxAbsPre = 0.0F;
            float maxAbsPost = 0.0F;
            float sumSquaresPost = 0.0F;
            for (int i = 0; i < kWaveformPoints; ++i) {
                const float sPre = state.voiceWaveformsPre[v][i];
                const float sPost = state.voiceWaveformsPost[v][i];
                maxAbsPre = std::max(maxAbsPre, std::abs(sPre));
                maxAbsPost = std::max(maxAbsPost, std::abs(sPost));
                sumSquaresPost += sPost * sPost;
            }

            state.voiceNormPre[v] =
                maxAbsPre > 1.0e-4F ? 1.0F / maxAbsPre : 0.0F;
            state.voiceNormPost[v] =
                maxAbsPost > 1.0e-4F ? 1.0F / maxAbsPost : 0.0F;

            if (kWaveformPoints > 0) {
                const float meanSquarePost = sumSquaresPost /
                                             static_cast<float>(
                                                 kWaveformPoints);
                state.voiceRmsPost[v] = meanSquarePost > 0.0F
                                             ? std::sqrt(meanSquarePost)
                                             : 0.0F;
            } else {
                state.voiceRmsPost[v] = 0.0F;
            }

            if (state.voiceNormPre[v] > 0.0F) {
                state.voicePeriodSamples[v] = estimateWaveformPeriod(
                    state.voiceWaveformsPre[v], kWaveformPoints);
            } else {
                state.voicePeriodSamples[v] = 0;
            }
        }

        return state;
    };

    AudioFrameState audioFrame = computeAudioFrameState(audioEngine_);
    audioFrame.modulesWithActiveAudio = &modulesWithActiveAudio_;
    audioFrame.moduleVoiceIndex = &moduleVoiceIndex_;
    audioFrame.connectionVisualSources = &connectionVisualSources_;

    auto isAudioConnection = [&modules](const rectai::Connection& conn) {
        const auto fromModuleIt = modules.find(conn.from_module_id);
        if (fromModuleIt != modules.end() && fromModuleIt->second != nullptr) {
            const auto& fromModule = *fromModuleIt->second;
            for (const auto& port : fromModule.output_ports()) {
                if (port.name == conn.from_port_name) {
                    return port.kind == rectai::PortSignalKind::kAudio;
                }
            }

            if (fromModule.produces_audio()) {
                return true;
            }
        }

        const auto toModuleIt = modules.find(conn.to_module_id);
        if (toModuleIt != modules.end() && toModuleIt->second != nullptr) {
            const auto& toModule = *toModuleIt->second;
            for (const auto& port : toModule.input_ports()) {
                if (port.name == conn.to_port_name) {
                    return port.kind == rectai::PortSignalKind::kAudio;
                }
            }

            if (toModule.consumes_audio()) {
                return true;
            }
        }

        return false;
    };

    auto getModuleVisualLevel = [](const rectai::AudioModule* module) {
        if (module == nullptr) {
            return 1.0F;
        }

        if (module->is<rectai::VolumeModule>()) {
            const float volume =
                module->GetParameterOrDefault("volume", 0.9F);
            return juce::jlimit(0.0F, 1.0F, volume);
        }

        if (module->is<rectai::SampleplayModule>() ||
            module->is<rectai::LoopModule>() ||
            module->is<rectai::InputModule>()) {
            const float amp = module->GetParameterOrDefault("amp", 1.0F);
            return juce::jlimit(0.0F, 1.0F, amp);
        }

        if (module->is<rectai::OscillatorModule>()) {
            const float gain = module->GetParameterOrDefault("gain", 0.5F);
            return juce::jlimit(0.0F, 1.0F, gain);
        }

        if (module->uses_gain_control()) {
            const float gain = module->GetParameterOrDefault("gain", 1.0F);
            return juce::jlimit(0.0F, 1.0F, gain);
        }

        return 1.0F;
    };

    auto drawWaveformOnLine = [&g](const juce::Point<float>& start,
                                   const juce::Point<float>& end,
                                   float amplitude, float thickness,
                                   const float* samples, int numSamples,
                                   float normalisation, int segments,
                                   bool tiled) {
        const juce::Point<float> delta = end - start;
        const float length =
            std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (length <= 0.0F) {
            return;
        }

        const juce::Point<float> tangent{delta.x / length,
                                          delta.y / length};
        const juce::Point<float> normal{-tangent.y, tangent.x};

        // Use a distance-based mapping from screen-space (pixels along
        // the line) to the waveform snapshot so that the visual pattern
        // (e.g. a saw wave shape) remains consistent regardless of the
        // physical length of the connection. A constant number of
        // samples per pixel avoids the "stretching/compressing" effect
        // observed when mapping the whole line [0,1] to the whole
        // buffer [0,numSamples).
        constexpr float kSamplesPerPixel = 0.8F;

        juce::Path path;
        for (int i = 0; i <= segments; ++i) {
            const float t = static_cast<float>(i) /
                            static_cast<float>(segments);
            const float baseX = start.x + delta.x * t;
            const float baseY = start.y + delta.y * t;
            int sampleIndex = 0;
            if (samples != nullptr && numSamples > 0) {
                if (tiled) {
                    const float distanceAlongLine = t * length;
                    const float samplePos =
                        distanceAlongLine * kSamplesPerPixel;
                    const int wrappedIndex =
                        static_cast<int>(samplePos) % numSamples;
                    sampleIndex = wrappedIndex >= 0
                                       ? wrappedIndex
                                       : wrappedIndex + numSamples;
                } else {
                    // For non-tiled sources (e.g. Sampleplay
                    // SoundFont output), map the entire buffer
                    // once along the line so that the visual
                    // curve reflects a contiguous window of audio
                    // history instead of a repeated period.
                    const float samplePos =
                        t * static_cast<float>(numSamples - 1);
                    int idx = static_cast<int>(samplePos);
                    if (idx < 0) {
                        idx = 0;
                    } else if (idx >= numSamples) {
                        idx = numSamples - 1;
                    }
                    sampleIndex = idx;
                }
            }

            float sampleValue =
                (samples != nullptr && numSamples > 0)
                    ? samples[sampleIndex]
                    : 0.0F;
            if (normalisation > 0.0F) {
                sampleValue *= normalisation;
            }

            // Apply a simple fade towards both ends of the line so
            // that the waveform visually tapers near its start and
            // end instead of stopping abruptly. The central section
            // (≈84% de la longitud) mantiene la amplitud completa.
            float taper = 1.0F;
            constexpr float kFadeSpan = 0.08F;  // ~8% at each side
            if (t < kFadeSpan) {
                const float u = t / kFadeSpan;
                taper = juce::jlimit(0.0F, 1.0F, u);
            } else if (t > 1.0F - kFadeSpan) {
                const float u = (1.0F - t) / kFadeSpan;
                taper = juce::jlimit(0.0F, 1.0F, u);
            }

            const float displacement = amplitude * sampleValue * taper;
            const juce::Point<float> offset{normal.x * displacement,
                                            normal.y * displacement};
            const float x = baseX + offset.x;
            const float y = baseY + offset.y;
            if (i == 0) {
                path.startNewSubPath(x, y);
            } else {
                path.lineTo(x, y);
            }
        }

        g.strokePath(path, juce::PathStrokeType(thickness));
    };

    // drawWaveformOnLine already handles all current use cases.
    // Keep the implementation minimal until we need curved paths.

    const GeometryCache geometry = buildGeometryCache();
    const auto& moduleToObjectId = geometry.moduleToObjectId;

    const std::unordered_set<std::int64_t> objectsWithOutgoingActiveConnection =
        computeObjectsWithOutgoingActiveConnection(moduleToObjectId);

    {
        juce::Graphics::ScopedSaveState clipState(g);

        // Clip central visuals (background pulses, connections) to the
        // musical area, but allow instruments themselves to be drawn
        // outside.
        juce::Path tableClip;
        const float tableRadiusClip =
            0.45F * std::min(bounds.getWidth(), bounds.getHeight());
        tableClip.addEllipse(centre.x - tableRadiusClip,
                             centre.y - tableRadiusClip,
                             tableRadiusClip * 2.0F,
                             tableRadiusClip * 2.0F);
        g.reduceClipRegion(tableClip);

        paintCentralPulses(g, bounds, centre);

        // -----------------------------------------------------------------
        // Connections: centre each object. All instruments keep their
        // own structural line to the master. When an instrument is
        // feeding another through an active connection, its line
        // remains visible but loses the animated flow pulse so that
        // only the downstream node appears as the active path to the
        // master bus.
        // -----------------------------------------------------------------
        for (const auto& [id, object] : objects) {
            if (object.logical_id() == rectai::MASTER_OUTPUT_ID) {
                continue;  // Output (master) is not drawn as a module.
            }

            const bool hasActiveOutgoingConnection =
                objectsWithOutgoingActiveConnection.find(id) !=
                objectsWithOutgoingActiveConnection.end();

            const auto modForConnectionIt =
                modules.find(object.logical_id());
            const rectai::AudioModule* moduleForConnection = nullptr;
            if (modForConnectionIt != modules.end() &&
                modForConnectionIt->second != nullptr) {
                moduleForConnection = modForConnectionIt->second.get();
            }

            const bool isGeneratorLike =
                moduleForConnection != nullptr &&
                moduleForConnection->type() ==
                    rectai::ModuleType::kGenerator;
            const bool isGlobalController =
                moduleForConnection != nullptr &&
                moduleForConnection->is_global_controller();

            const bool isAudioModule =
                moduleForConnection != nullptr &&
                (moduleForConnection->produces_audio() ||
                 moduleForConnection->consumes_audio());

            const bool isSampleplayModule =
                moduleForConnection != nullptr &&
                moduleForConnection->is<rectai::SampleplayModule>();

            const bool isLoopModule =
                moduleForConnection != nullptr &&
                moduleForConnection->is<rectai::LoopModule>();

            const bool isFilterModule =
                moduleForConnection != nullptr &&
                moduleForConnection->type() ==
                    rectai::ModuleType::kFilter;

            // Modules that expose a true volume-like control (right
            // arc) keep using their parameter to drive waveform
            // height. Modules without such a control (for example,
            // Filter, Delay, Modulator, WaveShaper, Output) derive
            // their visual level from the actual audio RMS measured
            // in the AudioEngine.
            const bool hasExplicitVolumeBar =
                moduleForConnection != nullptr &&
                (moduleForConnection->is<rectai::VolumeModule>() ||
                 moduleForConnection->is<rectai::OscillatorModule>() ||
                 moduleForConnection->is<rectai::SampleplayModule>() ||
                 moduleForConnection->is<rectai::LoopModule>() ||
                 moduleForConnection->is<rectai::InputModule>());

            const float visualLevel = getModuleVisualLevel(moduleForConnection);

#if !defined(NDEBUG)
            // Debug: track why radial lines may or may not be drawn
            // for generators, especially when investigating cases
            // where the Oscillator → master line appears only
            // briefly. Log only for audio modules inside the music
            // area to keep noise low.
            if (moduleForConnection != nullptr &&
                isInsideMusicArea(object) && isAudioModule) {
                juce::String msg("[rectai-core][paint-debug] radial id=");
                msg << object.logical_id().c_str()
                    << " isGenLike=" << (isGeneratorLike ? "1" : "0")
                    << " hasConn="
                    << (hasActiveOutgoingConnection ? "1" : "0")
                    << " globalCtl="
                    << (isGlobalController ? "1" : "0");
                juce::Logger::writeToLog(msg);
            }
#endif  // !defined(NDEBUG)

            // Only modules that actually carry audio (produce or
            // consume audio) draw a radial line to the master. Pure
            // control/MIDI modules such as Sequencer or LFO never
            // connect visually al centro.
            if (!isInsideMusicArea(object) || isGlobalController ||
                !isAudioModule) {
                continue;
            }

            // When an oscillator is feeding another module through an
            // active connection, it should not render a direct line to
            // the master centre; only the downstream module keeps the
            // visual link to the master bus.
            // Generative modules feeding another one hide their
            // direct visual link to the master; the downstream
            // module becomes the visible path.
            if (isGeneratorLike && hasActiveOutgoingConnection) {
                continue;
            }

            const auto centrePos = objectTableToScreen(object, bounds);
            const auto cx = centrePos.x;
            const auto cy = centrePos.y;

            juce::Line<float> line(centre.x, centre.y, cx, cy);

            const juce::Point<float> lineDelta =
                line.getEnd() - line.getStart();
            const float lineLength = std::sqrt(
                lineDelta.x * lineDelta.x + lineDelta.y * lineDelta.y);
            const int segmentsForWaveform =
                juce::jmax(32, static_cast<int>(lineLength / 4.0F));

            const bool lineCarriesAudio =
                // Sampleplay modules use the dedicated Sampleplay
                // waveform buffers instead of per-voice history and
                // do not rely on modulesWithActiveAudio_ to decide
                // whether su radial line carries sound.
                isSampleplayModule ||
                (audioFrame.modulesWithActiveAudio != nullptr &&
                 audioFrame.modulesWithActiveAudio->find(
                     object.logical_id()) !=
                     audioFrame.modulesWithActiveAudio->end());

            // Dim the line slightly when this instrument is feeding
            // another one, so the downstream instrument stands out.
            const float baseAlpha =
                hasActiveOutgoingConnection ? 0.4F : 0.7F;

            // Check if this line is marked for mute toggle (cut).
            const bool isMarkedForCut =
                touchCutObjects_.find(id) != touchCutObjects_.end();
            
            // Check if this line is being held for temporary mute with split rendering.
            const bool isBeingHeld = activeConnectionHold_.has_value() &&
                                     activeConnectionHold_->is_object_line &&
                                     activeConnectionHold_->object_id == id;
            
            if (isBeingHeld) {
                // Split rendering: waveform from object (module) to split point,
                // then dashed from split point to center (output/master).
                // Line goes from center (0.0) to object (1.0), so splitT is along that direction.
                const float splitT = activeConnectionHold_->split_point;
                const juce::Point<float> splitPoint{
                    juce::jmap(splitT, 0.0F, 1.0F, centre.x, cx),
                    juce::jmap(splitT, 0.0F, 1.0F, centre.y, cy)
                };

                bool drewWaveformSegment = false;

                // First segment: object to split (with waveform even if muted).
                // We support three families here:
                //  - Sampleplay modules: waveform from the corresponding
                //    audio connection tap.
                //  - Loop modules: per-module loop waveform history.
                //  - Procedural generators (e.g. Oscillator): per-voice
                //    post-filter waveform buffer.
                if (isSampleplayModule) {
                    float sampleplayRadialWave[kWaveformPoints]{};
                    float sampleplayRadialNorm = 0.0F;

                    std::string radialConnKey;
                    for (const auto& conn : scene_.connections()) {
                        if (conn.from_module_id != object.logical_id() ||
                            conn.to_module_id != rectai::MASTER_OUTPUT_ID) {
                            continue;
                        }

                        if (!isAudioConnection(conn)) {
                            continue;
                        }

                        // For hold-mute we always tap the connection,
                        // regardless of its persistent mute state; the
                        // audio path itself is gated separately.
                        radialConnKey = makeConnectionKey(conn);
                        break;
                    }

                    if (!radialConnKey.empty()) {
                        audioEngine_.getConnectionWaveformSnapshot(
                            radialConnKey, sampleplayRadialWave,
                            kWaveformPoints, 0.05);

                        float maxAbs = 0.0F;
                        for (int i = 0; i < kWaveformPoints; ++i) {
                            maxAbs = std::max(
                                maxAbs,
                                std::abs(sampleplayRadialWave[i]));
                        }

                        if (maxAbs > 1.0e-4F) {
                            sampleplayRadialNorm = 1.0F / maxAbs;
                        }
                    }

                    if (sampleplayRadialNorm > 0.0F) {
                        g.setColour(
                            juce::Colours::white.withAlpha(baseAlpha));
                        const float amplitudeLevel = visualLevel;
                        const float waveformAmplitude =
                            kWaveformAmplitudeScale * amplitudeLevel;
                        const float waveformThickness = 1.4F;

                        const juce::Point<float> delta =
                            splitPoint - juce::Point<float>{cx, cy};
                        const float segmentLineLength =
                            std::sqrt(delta.x * delta.x +
                                      delta.y * delta.y);
                        const int segmentsForWaveformHold =
                            static_cast<int>(segmentLineLength / 5.0F);

                        drawWaveformOnLine(
                            {cx, cy}, splitPoint, waveformAmplitude,
                            waveformThickness, sampleplayRadialWave,
                            kWaveformPoints, sampleplayRadialNorm,
                            juce::jmax(1, segmentsForWaveformHold),
                            /*tiled=*/false);
                        drewWaveformSegment = true;
                    }
                } else if (isLoopModule) {
                    float loopRadialWave[kWaveformPoints]{};
                    float loopRadialNorm = 0.0F;

                    audioEngine_.getLoopModuleWaveformSnapshot(
                        object.logical_id(), loopRadialWave,
                        kWaveformPoints, 0.05);

                    float maxAbs = 0.0F;
                    for (int i = 0; i < kWaveformPoints; ++i) {
                        maxAbs = std::max(maxAbs,
                                          std::abs(loopRadialWave[i]));
                    }

                    if (maxAbs > 1.0e-4F) {
                        loopRadialNorm = 1.0F / maxAbs;
                    }

                    if (loopRadialNorm > 0.0F) {
                        g.setColour(
                            juce::Colours::white.withAlpha(baseAlpha));
                        const float amplitudeLevel = visualLevel;
                        const float waveformAmplitude =
                            kWaveformAmplitudeScale * amplitudeLevel;
                        const float waveformThickness = 1.2F;

                        const juce::Point<float> delta =
                            splitPoint - juce::Point<float>{cx, cy};
                        const float segmentLineLength =
                            std::sqrt(delta.x * delta.x +
                                      delta.y * delta.y);
                        const int segmentsForWaveformHold =
                            static_cast<int>(segmentLineLength / 5.0F);

                        drawWaveformOnLine(
                            {cx, cy}, splitPoint, waveformAmplitude,
                            waveformThickness, loopRadialWave,
                            kWaveformPoints, loopRadialNorm,
                            juce::jmax(1, segmentsForWaveformHold),
                            /*tiled=*/false);
                        drewWaveformSegment = true;
                    }
                } else {
                    const auto* voiceMap = audioFrame.moduleVoiceIndex;
                    const auto voiceIt =
                        (voiceMap != nullptr)
                            ? voiceMap->find(object.logical_id())
                            : std::unordered_map<std::string, int>::const_iterator{};
                    const int voiceIndex =
                        (voiceMap != nullptr &&
                         voiceIt != voiceMap->end())
                            ? voiceIt->second
                            : -1;

                    if (voiceIndex >= 0 &&
                        voiceIndex < AudioEngine::kMaxVoices &&
                        audioFrame.voiceNormPost[voiceIndex] > 0.0F) {
                        g.setColour(
                            juce::Colours::white.withAlpha(baseAlpha));

                        float amplitudeLevel = visualLevel;
                        if (!hasExplicitVolumeBar) {
                            const float rms = audioFrame.voiceRmsPost[voiceIndex];
                            if (rms > 0.0F) {
                                amplitudeLevel = juce::jlimit(
                                    0.0F, 1.0F, rms / 0.7071F);
                            } else {
                                amplitudeLevel = 0.0F;
                            }
                        }

                        const float waveformAmplitude =
                            kWaveformAmplitudeScale * amplitudeLevel;
                        const float waveformThickness = 1.4F;

                        const juce::Point<float> delta =
                            splitPoint - juce::Point<float>{cx, cy};
                        const float segmentLineLength =
                            std::sqrt(delta.x * delta.x +
                                      delta.y * delta.y);
                        const int segmentsForWaveformHold =
                            static_cast<int>(segmentLineLength / 5.0F);
                        const int periodSamples =
                            audioFrame.voicePeriodSamples[voiceIndex] > 0
                                ? audioFrame.voicePeriodSamples[voiceIndex]
                                : kWaveformPoints;
                        float normalisation =
                            audioFrame.voiceNormPost[voiceIndex];
                        if (!hasExplicitVolumeBar) {
                            normalisation = 1.0F;
                        }
                        drawWaveformOnLine(
                            {cx, cy}, splitPoint, waveformAmplitude,
                            waveformThickness,
                            audioFrame.voiceWaveformsPost[voiceIndex],
                            periodSamples, normalisation,
                            juce::jmax(1, segmentsForWaveformHold),
                            /*tiled=*/true);
                        drewWaveformSegment = true;
                    }
                }

                if (!drewWaveformSegment) {
                    g.setColour(juce::Colours::white.withAlpha(baseAlpha));
                    g.drawLine(juce::Line<float>({cx, cy}, splitPoint),
                               1.2F);
                }

                // Second segment: split to center (dashed, muted/silenced output).
                const float dashLengths[] = {6.0F, 4.0F};
                g.setColour(
                    juce::Colours::white.withAlpha(baseAlpha * 0.6F));
                g.drawDashedLine(juce::Line<float>(splitPoint, centre),
                                 dashLengths,
                                 static_cast<int>(
                                     std::size(dashLengths)),
                                 1.2F);
                continue;
            }

            // A radial line is visually muted when its implicit
            // connection to the master Output (MASTER_OUTPUT_ID) is
            // muted at the connection level. We derive that state by
            // checking for a module -> MASTER_OUTPUT_ID connection in
            // mutedConnections_. Additionally, if that route is a
            // hardlink, we reflect it in the radial colour so that
            // hardlinked paths to the master are visually prioritised
            // over purely dynamic ones.
            bool isRadialMuted = false;
            bool hasHardlinkToMaster = false;
            {
                for (const auto& conn : scene_.connections()) {
                    if (conn.from_module_id != object.logical_id() ||
                        conn.to_module_id != rectai::MASTER_OUTPUT_ID) {
                        continue;
                    }

                    if (conn.is_hardlink) {
                        hasHardlinkToMaster = true;
                    }

                    const std::string key = makeConnectionKey(conn);
                    if (mutedConnections_.find(key) !=
                        mutedConnections_.end()) {
                        isRadialMuted = true;
                        // Still continue scanning so that
                        // hasHardlinkToMaster can be set even when
                        // the first matching connection is muted.
                    }
                }
            }

            // Detect if this module (typically a Filter) is being
            // driven directamente por Sampleplay a efectos de
            // compatibilidad con escenas antiguas. La lógica de
            // visualización de radiales se basa ahora en una señal
            // agregada de todas las entradas de audio (ver más
            // abajo), pero mantenemos este flag para reutilizar la
            // waveform de Sampleplay cuando proceda.
            bool hasSampleplayUpstream = false;
            std::string sampleplayUpstreamConnKey;
            if (isFilterModule) {
                for (const auto& conn : scene_.connections()) {
                    if (!isAudioConnection(conn) ||
                        conn.to_module_id != object.logical_id()) {
                        continue;
                    }

                    const auto modItFrom =
                        modules.find(conn.from_module_id);
                    if (modItFrom == modules.end() ||
                        modItFrom->second == nullptr ||
                        !modItFrom->second
                             ->is<rectai::SampleplayModule>()) {
                        continue;
                    }

                    const auto fromObjIdIt =
                        moduleToObjectId.find(conn.from_module_id);
                    const auto toObjIdIt =
                        moduleToObjectId.find(conn.to_module_id);
                    if (fromObjIdIt == moduleToObjectId.end() ||
                        toObjIdIt == moduleToObjectId.end()) {
                        continue;
                    }

                    const auto fromObjIt =
                        objects.find(fromObjIdIt->second);
                    const auto toObjIt =
                        objects.find(toObjIdIt->second);
                    if (fromObjIt == objects.end() ||
                        toObjIt == objects.end()) {
                        continue;
                    }

                    const auto& fromObj = fromObjIt->second;
                    const auto& toObj = toObjIt->second;
                    if (!isInsideMusicArea(fromObj) ||
                        !isInsideMusicArea(toObj)) {
                        continue;
                    }

                    if (!conn.is_hardlink &&
                        !isConnectionGeometricallyActive(fromObj,
                                                         toObj)) {
                        continue;
                    }

                    const std::string key = makeConnectionKey(conn);
                    const bool connIsMuted =
                        mutedConnections_.find(key) !=
                        mutedConnections_.end();
                    if (connIsMuted) {
                        continue;
                    }

                    hasSampleplayUpstream = true;
                    sampleplayUpstreamConnKey = key;
                    break;
                }
            }

            // Helper to compute an aggregated input waveform for
            // modules that consumen audio desde varias conexiones.
            // Suma las waveforms de todas las conexiones de audio
            // activas que terminan en este módulo (después de mutes
            // y geometría) utilizando los taps por conexión del
            // AudioEngine.
            auto computeAggregatedIncomingWaveform =
                [&](float* dst, float& norm, float& rms) -> bool {
                std::fill(dst, dst + kWaveformPoints, 0.0F);
                norm = 0.0F;
                rms = 0.0F;

                float temp[kWaveformPoints]{};
                int activeConnections = 0;

                for (const auto& conn : scene_.connections()) {
                    if (!isAudioConnection(conn) ||
                        conn.to_module_id != object.logical_id()) {
                        continue;
                    }

                    const auto fromObjIdIt =
                        moduleToObjectId.find(conn.from_module_id);
                    const auto toObjIdIt =
                        moduleToObjectId.find(conn.to_module_id);
                    if (fromObjIdIt == moduleToObjectId.end() ||
                        toObjIdIt == moduleToObjectId.end()) {
                        continue;
                    }

                    const auto fromObjIt =
                        objects.find(fromObjIdIt->second);
                    const auto toObjIt =
                        objects.find(toObjIdIt->second);
                    if (fromObjIt == objects.end() ||
                        toObjIt == objects.end()) {
                        continue;
                    }

                    const auto& fromObj = fromObjIt->second;
                    const auto& toObj = toObjIt->second;
                    if (!isInsideMusicArea(fromObj) ||
                        !isInsideMusicArea(toObj)) {
                        continue;
                    }

                    if (!conn.is_hardlink &&
                        !isConnectionGeometricallyActive(fromObj,
                                                         toObj)) {
                        continue;
                    }

                    const std::string key = makeConnectionKey(conn);
                    if (mutedConnections_.find(key) !=
                        mutedConnections_.end()) {
                        continue;
                    }

                    // Sólo consideramos conexiones que tienen un
                    // tap configurado en el motor; esto evita pedir
                    // snapshots para claves que no se están
                    // actualizando en audio.
                    const auto* vsMap =
                        audioFrame.connectionVisualSources;
                    if (vsMap == nullptr ||
                        vsMap->find(key) == vsMap->end()) {
                        continue;
                    }

                    audioEngine_.getConnectionWaveformSnapshot(
                        key, temp, kWaveformPoints, 0.05);

                    for (int i = 0; i < kWaveformPoints; ++i) {
                        dst[i] += temp[i];
                    }

                    ++activeConnections;
                }

                if (activeConnections <= 0) {
                    return false;
                }

                float maxAbs = 0.0F;
                float sumSquares = 0.0F;
                for (int i = 0; i < kWaveformPoints; ++i) {
                    const float v = dst[i];
                    maxAbs = std::max(maxAbs, std::abs(v));
                    sumSquares += v * v;
                }

                if (maxAbs <= 1.0e-4F) {
                    return false;
                }

                norm = 1.0F / maxAbs;
                if (kWaveformPoints > 0) {
                    const float meanSquare =
                        sumSquares /
                        static_cast<float>(kWaveformPoints);
                    rms = meanSquare > 0.0F
                              ? std::sqrt(meanSquare)
                              : 0.0F;
                } else {
                    rms = 0.0F;
                }

                return true;
            };

            // For Sampleplay-related radials, derive the waveform
            // from the same per-connection taps used for module→
            // module edges so that Sampleplay → Master and
            // Sampleplay → Filter share a single underlying
            // waveform source.
            float sampleplayRadialWave[kWaveformPoints]{};
            float sampleplayRadialNorm = 0.0F;

            if (!isRadialMuted &&
                (isSampleplayModule ||
                 (isFilterModule && hasSampleplayUpstream))) {
                std::string radialConnKey;

                if (isSampleplayModule) {
                    // Use the implicit Sampleplay → Output (-1)
                    // audio connection for this module.
                    for (const auto& conn : scene_.connections()) {
                        if (conn.from_module_id !=
                                object.logical_id() ||
                            conn.to_module_id != rectai::MASTER_OUTPUT_ID) {
                            continue;
                        }

                        if (!isAudioConnection(conn)) {
                            continue;
                        }

                        const std::string key =
                            makeConnectionKey(conn);
                        if (mutedConnections_.find(key) !=
                            mutedConnections_.end()) {
                            continue;
                        }

                        radialConnKey = key;
                        break;
                    }
                } else if (isFilterModule && hasSampleplayUpstream &&
                           !sampleplayUpstreamConnKey.empty()) {
                    // For filters driven directly by Sampleplay,
                    // reuse the upstream Sampleplay → Filter
                    // connection tap so that the filter radial
                    // reflects the same waveform as that edge.
                    radialConnKey = sampleplayUpstreamConnKey;
                }

                if (!radialConnKey.empty()) {
                    audioEngine_.getConnectionWaveformSnapshot(
                        radialConnKey, sampleplayRadialWave,
                        kWaveformPoints, 0.05);

                    float maxAbs = 0.0F;
                    for (int i = 0; i < kWaveformPoints; ++i) {
                        maxAbs = std::max(
                            maxAbs,
                            std::abs(sampleplayRadialWave[i]));
                    }

                    if (maxAbs > 1.0e-4F) {
                        sampleplayRadialNorm = 1.0F / maxAbs;
                    }
                }
            }

            const bool sampleplayRadialActive =
                sampleplayRadialNorm > 0.0F && !isRadialMuted &&
                (isSampleplayModule ||
                 (isFilterModule && hasSampleplayUpstream));

            if (sampleplayRadialActive) {
                const auto baseColour = hasHardlinkToMaster
                                            ? juce::Colours::red
                                            : juce::Colours::white;
                const auto radialColour =
                    isMarkedForCut
                        ? juce::Colours::yellow.withAlpha(0.9F)
                        : baseColour.withAlpha(baseAlpha);
                g.setColour(radialColour);
                const float amplitudeLevel =
                    isSampleplayModule ? visualLevel : 1.0F;
                const float waveformAmplitude =
                    kWaveformAmplitudeScale * amplitudeLevel;
                const float waveformThickness = 1.4F;
                drawWaveformOnLine(
                    line.getEnd(), line.getStart(),
                    waveformAmplitude, waveformThickness,
                    sampleplayRadialWave, kWaveformPoints,
                    sampleplayRadialNorm, segmentsForWaveform,
                    /*tiled=*/false);
                continue;
            }

            // For Loop modules, derive the radial waveform from a
            // per-module history buffer maintained by the
            // AudioEngine. This allows each Loop radial to display
            // only the signal generated by its own sample (post
            // gain/envelope) en lugar de compartir la mezcla global
            // con otros loops.
            float loopRadialWave[kWaveformPoints]{};
            float loopRadialNorm = 0.0F;

            if (!isRadialMuted && isLoopModule && lineCarriesAudio) {
                audioEngine_.getLoopModuleWaveformSnapshot(
                    object.logical_id(), loopRadialWave,
                    kWaveformPoints, 0.05);

                float maxAbs = 0.0F;
                for (int i = 0; i < kWaveformPoints; ++i) {
                    maxAbs = std::max(maxAbs,
                                      std::abs(loopRadialWave[i]));
                }

                if (maxAbs > 1.0e-4F) {
                    loopRadialNorm = 1.0F / maxAbs;
                }
            }

            const bool loopRadialActive =
                loopRadialNorm > 0.0F && !isRadialMuted &&
                isLoopModule && lineCarriesAudio;

            if (loopRadialActive) {
                const auto baseColour = hasHardlinkToMaster
                                            ? juce::Colours::red
                                            : juce::Colours::white;
                const auto radialColour =
                    isMarkedForCut
                        ? juce::Colours::yellow.withAlpha(0.9F)
                        : baseColour.withAlpha(baseAlpha);
                g.setColour(radialColour);
                const float amplitudeLevel = visualLevel;
                const float waveformAmplitude =
                    kWaveformAmplitudeScale * amplitudeLevel;
                // Use a slightly thinner stroke so that Loop
                // radials visually match the perceived thickness
                // of Oscillator radials.
                const float waveformThickness = 1.2F;
                drawWaveformOnLine(
                    line.getEnd(), line.getStart(),
                    waveformAmplitude, waveformThickness,
                    loopRadialWave, kWaveformPoints, loopRadialNorm,
                    segmentsForWaveform,
                    /*tiled=*/false);
                continue;
            }

            if (lineCarriesAudio && !isRadialMuted && isAudioModule) {
                bool drewRadial = false;

                // Para módulos consumidores de audio sin barra de
                // volumen propia (p.ej. filtros), intentamos
                // representar primero la suma real de todas las
                // entradas de audio que les llegan a través del
                // grafo (Osc, Sampleplay, Loop, etc.).
                if (!hasExplicitVolumeBar && !isSampleplayModule &&
                    !isLoopModule) {
                    float aggWave[kWaveformPoints]{};
                    float aggNorm = 0.0F;
                    float aggRms = 0.0F;
                    if (computeAggregatedIncomingWaveform(
                            aggWave, aggNorm, aggRms)) {
                        const auto baseColour = hasHardlinkToMaster
                                                    ? juce::Colours::red
                                                    : juce::Colours::white;
                        const auto waveformColour =
                            isMarkedForCut
                                ? juce::Colours::yellow.withAlpha(0.9F)
                                : baseColour.withAlpha(baseAlpha);
                        g.setColour(waveformColour);

                        float amplitudeLevel = 0.0F;
                        if (aggRms > 0.0F) {
                            amplitudeLevel = juce::jlimit(
                                0.0F, 1.0F, aggRms / 0.7071F);
                        }

                        const float waveformAmplitude =
                            kWaveformAmplitudeScale * amplitudeLevel;
                        const float waveformThickness = 1.4F;
                        drawWaveformOnLine(
                            line.getEnd(), line.getStart(),
                            waveformAmplitude, waveformThickness,
                            aggWave, kWaveformPoints,
                            /*normalisation=*/1.0F,
                            segmentsForWaveform,
                            /*tiled=*/false);
                        drewRadial = true;
                    }
                }

                if (!drewRadial) {
                    const auto* voiceMap = audioFrame.moduleVoiceIndex;
                    const auto voiceIt =
                        (voiceMap != nullptr)
                            ? voiceMap->find(object.logical_id())
                            : std::unordered_map<std::string, int>::const_iterator{};
                    const int voiceIndex =
                        (voiceMap != nullptr &&
                         voiceIt != voiceMap->end())
                            ? voiceIt->second
                            : -1;

                    if (voiceIndex >= 0 &&
                        voiceIndex < AudioEngine::kMaxVoices &&
                        audioFrame.voiceNormPost[voiceIndex] > 0.0F) {
                        const auto baseColour = hasHardlinkToMaster
                                                    ? juce::Colours::red
                                                    : juce::Colours::white;
                        const auto waveformColour =
                            isMarkedForCut
                                ? juce::Colours::yellow.withAlpha(0.9F)
                                : baseColour.withAlpha(baseAlpha);
                        g.setColour(waveformColour);
                        float amplitudeLevel = visualLevel;

                        // If this module does not have an explicit
                        // volume bar, derive the visual level from
                        // the RMS of the corresponding audio voice
                        // so that filters/FX reflect their actual
                        // output level instead of a fixed height.
                        if (!hasExplicitVolumeBar) {
                            const float rms =
                                audioFrame.voiceRmsPost[voiceIndex];
                            if (rms > 0.0F) {
                                // Normalise against a full-scale sine
                                // (~0.707 RMS) and clamp to [0,1].
                                amplitudeLevel = juce::jlimit(
                                    0.0F, 1.0F, rms / 0.7071F);
                            } else {
                                amplitudeLevel = 0.0F;
                            }
                        }

                        const float waveformAmplitude =
                            kWaveformAmplitudeScale * amplitudeLevel;
                        const float waveformThickness = 1.4F;
                        const int periodSamples =
                            audioFrame.voicePeriodSamples[voiceIndex] > 0
                                ? audioFrame.voicePeriodSamples[voiceIndex]
                                : kWaveformPoints;
                        float normalisation =
                            audioFrame.voiceNormPost[voiceIndex];
                        if (!hasExplicitVolumeBar) {
                            normalisation = 1.0F;
                        }
                        drawWaveformOnLine(
                            line.getEnd(), line.getStart(),
                            waveformAmplitude, waveformThickness,
                            audioFrame.voiceWaveformsPost[voiceIndex],
                            periodSamples, normalisation,
                            segmentsForWaveform,
                            /*tiled=*/true);
                        drewRadial = true;
                    }
                }

                if (drewRadial) {
                    continue;
                }
            }

            // Fallback for non-audio or silent lines. Preserve the
            // existing style (solid vs dashed) and only change the
            // colour when the line is marked for a pending
            // mute/unmute operation.
            const auto baseRadialColour = hasHardlinkToMaster
                                               ? juce::Colours::red
                                               : juce::Colours::white;
            const auto radialFallbackColour =
                isMarkedForCut
                    ? juce::Colours::yellow.withAlpha(0.9F)
                    : baseRadialColour.withAlpha(baseAlpha);
            g.setColour(radialFallbackColour);
            if (isRadialMuted) {
                const float dashLengths[] = {6.0F, 4.0F};
                g.drawDashedLine(
                    line, dashLengths,
                    static_cast<int>(std::size(dashLengths)), 1.2F);
            } else {
                g.drawLine(line, 1.2F);
            }

            // Flow pulse travelling from node to centre to suggest
            // direction.
            const float t = static_cast<float>(std::fmod(
                connectionFlowPhase_ +
                    0.25 * static_cast<double>(id % 4),
                1.0));
            juce::ignoreUnused(t, cx, cy, centre);
        }

        // -----------------------------------------------------------------
        // Connections between modules (Scene::connections) as edges with a
        // small animated pulse suggesting signal flow. Both dynamic and
        // hardlink connections are rendered as straight segments between
        // modules; dynamic connections are considered active only when the
        // target lies inside the geometric cone of the source (see
        // isConnectionGeometricallyActive), while hardlink connections
        // remain active regardless of that angular constraint and are drawn
        // in red instead of white.
        // -----------------------------------------------------------------
        int connectionIndex = 0;
        for (const auto& conn : scene_.connections()) {
            // Do not draw explicit module-to-module lines that
            // target the invisible Output/master module (id
            // MASTER_OUTPUT_ID). The visual representation of a
            // module's route to the master is always the radial
            // line from the object to the centre of the table; an
            // additional connection line to the hidden Output
            // object would duplicate that path visually.
            if (conn.to_module_id == rectai::MASTER_OUTPUT_ID) {
                ++connectionIndex;
                continue;
            }

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

            const auto fromPos = objectTableToScreen(fromObj, bounds);
            const auto fx = fromPos.x;
            const auto fy = fromPos.y;
            const auto toPos = objectTableToScreen(toObj, bounds);
            const auto tx = toPos.x;
            const auto ty = toPos.y;

            const juce::Point<float> p1{fx, fy};
            const juce::Point<float> p2{tx, ty};
            const std::string connKey = makeConnectionKey(conn);

            // A connection is considered muted either when it is
            // explicitly in mutedConnections_, or when the source
            // module has all its routes to the master Output (-1)
            // muted (i.e. its implicit module→master connection is
            // muted). This ensures that muting a module's path to
            // the master also visually mutes any downstream
            // module→module edges originating from it.
            bool sourceMutedToMaster = false;
            {
                bool hasMasterRoute = false;
                bool masterRouteMuted = true;

                // Scan all connections from this source module to the
                // master Output (-1). If at least one such route exists
                // and any of them is not muted, then the source is
                // considered audible to the master.
                for (const auto& mconn : scene_.connections()) {
                    if (mconn.from_module_id != conn.from_module_id ||
                        mconn.to_module_id != rectai::MASTER_OUTPUT_ID) {
                        continue;
                    }

                    hasMasterRoute = true;
                    const std::string mkey = makeConnectionKey(mconn);
                    const bool connIsMuted =
                        mutedConnections_.find(mkey) !=
                        mutedConnections_.end();
                    if (!connIsMuted) {
                        masterRouteMuted = false;
                        break;
                    }
                }

                sourceMutedToMaster = hasMasterRoute && masterRouteMuted;
            }
            const bool explicitlyMuted =
                mutedConnections_.find(connKey) !=
                mutedConnections_.end();

            const bool audioConnEndpointsActive =
                isAudioConnection(conn) &&
                audioFrame.modulesWithActiveAudio != nullptr &&
                (audioFrame.modulesWithActiveAudio->find(
                     conn.from_module_id) !=
                     audioFrame.modulesWithActiveAudio->end() ||
                 audioFrame.modulesWithActiveAudio->find(
                     conn.to_module_id) !=
                     audioFrame.modulesWithActiveAudio->end());

            // Check if either endpoint of this connection is a
            // Sampleplay module so we can use the dedicated
            // Sampleplay waveform history when drawing its audio
            // path, instead of looking for an oscillator voice.
            const rectai::AudioModule* fromModulePtr = nullptr;
            const rectai::AudioModule* toModulePtr = nullptr;
            if (const auto it = modules.find(conn.from_module_id);
                it != modules.end() && it->second != nullptr) {
                fromModulePtr = it->second.get();
            }
            if (const auto it = modules.find(conn.to_module_id);
                it != modules.end() && it->second != nullptr) {
                toModulePtr = it->second.get();
            }

            const bool involvesSampleplay =
                (fromModulePtr != nullptr &&
                 fromModulePtr->is<rectai::SampleplayModule>()) ||
                (toModulePtr != nullptr &&
                 toModulePtr->is<rectai::SampleplayModule>());

            // For connections that involve Sampleplay, consider only
            // explicit mutes. El mute "a nivel de módulo" hacia el
            // master (sourceMutedToMaster) no debe apagar su conexión
            // Sampleplay → Filter.
            const bool isMutedConnection =
                explicitlyMuted ||
                (!involvesSampleplay && sourceMutedToMaster);

            // Connections that involve Sampleplay rely on the
            // dedicated Sampleplay buffer and per-connection taps
            // instead of the generator-based active set. Consider
            // them audio-bearing whenever they are not muted.
            const bool audioConn =
                isAudioConnection(conn) &&
                (!involvesSampleplay ? audioConnEndpointsActive
                                     : !explicitlyMuted);

            const float fromLevel = getModuleVisualLevel(fromModulePtr);
            const float toLevel = getModuleVisualLevel(toModulePtr);

            // Check if this connection is marked for mute toggle.
            const bool isConnectionMarkedForCut =
                touchCutConnections_.find(connKey) !=
                touchCutConnections_.end();

            const juce::Colour activeColour =
                isConnectionMarkedForCut
                    ? juce::Colours::yellow.withAlpha(0.9F)
                    : (conn.is_hardlink
                           ? juce::Colours::red.withAlpha(0.8F)
                           : juce::Colours::white.withAlpha(0.7F));
            
            // Helper to obtain a representative waveform for this
            // connection. We first try the dedicated per-connection
            // tap in the AudioEngine; if that yields a silent buffer
            // (e.g. for placeholder FX modules that still do not
            // generate audio on their own), we fall back to an
            // aggregated view of all audio inputs feeding the source
            // module. This makes chains such as Osc -> Delay ->
            // Master display a consistent waveform on both
            // connections even when Delay is still a pass-through in
            // the DSP layer.
            auto fetchConnectionWaveformOrAggregate =
                [&](const std::string& key, const std::string& fromModuleId,
                    float* dst, float& norm, float& rms) -> bool {
                std::fill(dst, dst + kWaveformPoints, 0.0F);
                norm = 0.0F;
                rms = 0.0F;

                // First try the direct per-connection tap.
                audioEngine_.getConnectionWaveformSnapshot(
                    key, dst, kWaveformPoints, 0.05);

                float maxAbs = 0.0F;
                float sumSquares = 0.0F;
                for (int i = 0; i < kWaveformPoints; ++i) {
                    const float v = dst[i];
                    maxAbs = std::max(maxAbs, std::abs(v));
                    sumSquares += v * v;
                }

                if (maxAbs > 1.0e-4F) {
                    norm = 1.0F / maxAbs;
                    if (kWaveformPoints > 0) {
                        const float meanSquare =
                            sumSquares /
                            static_cast<float>(kWaveformPoints);
                        rms = meanSquare > 0.0F
                                  ? std::sqrt(meanSquare)
                                  : 0.0F;
                    }
                    return true;
                }

                // Fallback: aggregate all active audio connections
                // that feed the source module. This leverages the
                // same per-connection taps used elsewhere but sums
                // them so that consumer-only modules (e.g. Delay
                // before it has a dedicated DSP path) can still show
                // a meaningful waveform on their outgoing edges.
                float temp[kWaveformPoints]{};
                int activeConnections = 0;

                for (const auto& ic : scene_.connections()) {
                    if (!isAudioConnection(ic) ||
                        ic.to_module_id != fromModuleId) {
                        continue;
                    }

                    const auto fromObjIdIt =
                        moduleToObjectId.find(ic.from_module_id);
                    const auto toObjIdIt =
                        moduleToObjectId.find(ic.to_module_id);
                    if (fromObjIdIt == moduleToObjectId.end() ||
                        toObjIdIt == moduleToObjectId.end()) {
                        continue;
                    }

                    const auto fromObjIt =
                        objects.find(fromObjIdIt->second);
                    const auto toObjIt =
                        objects.find(toObjIdIt->second);
                    if (fromObjIt == objects.end() ||
                        toObjIt == objects.end()) {
                        continue;
                    }

                    const auto& fromObj = fromObjIt->second;
                    const auto& toObj = toObjIt->second;
                    if (!isInsideMusicArea(fromObj) ||
                        !isInsideMusicArea(toObj)) {
                        continue;
                    }

                    if (!ic.is_hardlink &&
                        !isConnectionGeometricallyActive(fromObj,
                                                         toObj)) {
                        continue;
                    }

                    const std::string upstreamKey =
                        makeConnectionKey(ic);
                    if (mutedConnections_.find(upstreamKey) !=
                        mutedConnections_.end()) {
                        continue;
                    }

                    const auto* vsMap =
                        audioFrame.connectionVisualSources;
                    if (vsMap == nullptr ||
                        vsMap->find(upstreamKey) == vsMap->end()) {
                        continue;
                    }

                    audioEngine_.getConnectionWaveformSnapshot(
                        upstreamKey, temp, kWaveformPoints, 0.05);

                    for (int i = 0; i < kWaveformPoints; ++i) {
                        dst[i] += temp[i];
                    }

                    ++activeConnections;
                }

                if (activeConnections <= 0) {
                    return false;
                }

                float maxAbsAgg = 0.0F;
                float sumSquaresAgg = 0.0F;
                for (int i = 0; i < kWaveformPoints; ++i) {
                    const float v = dst[i];
                    maxAbsAgg = std::max(maxAbsAgg, std::abs(v));
                    sumSquaresAgg += v * v;
                }

                if (maxAbsAgg <= 1.0e-4F) {
                    return false;
                }

                norm = 1.0F / maxAbsAgg;
                if (kWaveformPoints > 0) {
                    const float meanSquare =
                        sumSquaresAgg /
                        static_cast<float>(kWaveformPoints);
                    rms = meanSquare > 0.0F
                              ? std::sqrt(meanSquare)
                              : 0.0F;
                } else {
                    rms = 0.0F;
                }

                return true;
            };

            // Check if this connection is being held for temporary mute with split rendering.
            const bool isBeingHeld = activeConnectionHold_.has_value() &&
                                     !activeConnectionHold_->is_object_line &&
                                     activeConnectionHold_->connection_key == connKey;

            if (isBeingHeld) {
                // Split rendering: waveform from source to split point,
                // then dashed from split point to destination.
                const float splitT = activeConnectionHold_->split_point;
                const juce::Point<float> splitPoint{
                    juce::jmap(splitT, 0.0F, 1.0F, p1.x, p2.x),
                    juce::jmap(splitT, 0.0F, 1.0F, p1.y, p2.y)
                };
                
                // First segment: source to split (with waveform even
                // if the connection is temporarily held). The
                // waveform comes from the per-connection tap in the
                // audio engine, which already encodes whether this
                // connection observes a pre/post voice or Sampleplay
                // signal.
                float tempWave[kWaveformPoints]{};
                float norm = 0.0F;
                float rms = 0.0F;

                const bool hasWave = fetchConnectionWaveformOrAggregate(
                    connKey, conn.from_module_id, tempWave, norm, rms);

                bool drewWave = false;
                if (hasWave && rms > 0.0F) {
                    g.setColour(activeColour);
                    const float amplitudeLevel = juce::jlimit(
                        0.0F, 1.0F, rms / 0.7071F);
                    const float waveformAmplitude =
                        kWaveformAmplitudeScale * amplitudeLevel;
                    const float waveformThickness = conn.is_hardlink ? 1.2F : 1.2F;
                    const juce::Point<float> delta = splitPoint - p1;
                    const float lineLength = std::sqrt(
                        delta.x * delta.x + delta.y * delta.y);
                    const int rawSegments =
                        static_cast<int>(lineLength / 5.0F);
                    const int segmentsForWaveform =
                        juce::jmax(1, juce::jmin(rawSegments, 80));
                    // Avoid per-frame period estimation here – for the
                    // held connection preview a single non-tiled buffer
                    // span is visually sufficient and cheaper to render.
                    const int periodSamples = kWaveformPoints;

                    const bool tiled = !involvesSampleplay;
                    drawWaveformOnLine(
                        p1, splitPoint, waveformAmplitude,
                        waveformThickness, tempWave,
                        periodSamples, /*normalisation=*/1.0F,
                        segmentsForWaveform,
                        /*tiled=*/tiled);
                    drewWave = true;
                }

                if (!drewWave) {
                    g.setColour(activeColour);
                    g.drawLine(juce::Line<float>(p1, splitPoint), 1.2F);
                }
                
                // Second segment: split to destination (dashed, muted).
                const float dashLengths[] = {6.0F, 4.0F};
                g.setColour(activeColour.withAlpha(0.6F));
                g.drawDashedLine(juce::Line<float>(splitPoint, p2), dashLengths,
                                 static_cast<int>(std::size(dashLengths)), 1.2F);
                continue;
            }

            if (isMutedConnection) {
                // Muted connection: render as a dashed straight line
                // between the two nodes and omit the animated flow
                // pulse.
                const float dashLengths[] = {6.0F, 4.0F};
                g.setColour(activeColour.withAlpha(0.6F));
                g.drawDashedLine(juce::Line<float>(p1, p2), dashLengths,
                                 static_cast<int>(
                                     std::size(dashLengths)),
                                 1.2F);
            } else if (conn.is_hardlink) {
                // Hardlink: straight segment with a pulse travelling
                // directly from source to destination.
                g.setColour(activeColour);
                bool drewWave = false;

                if (audioConn) {
                    float tempWave[kWaveformPoints]{};
                    float norm = 0.0F;
                    float rms = 0.0F;

                    const bool hasWave =
                        fetchConnectionWaveformOrAggregate(
                            connKey, conn.from_module_id, tempWave,
                            norm, rms);

                    // Skip waveform rendering for connections with a very
                    // low effective level; a straight line is visually
                    // sufficient and significantly cheaper to rasterise.
                    if (hasWave && rms > 0.0F) {
                        const float amplitudeLevel = juce::jlimit(
                            0.0F, 1.0F, rms / 0.7071F);
                        const float waveformAmplitude =
                            kWaveformAmplitudeScale * amplitudeLevel;
                        const float waveformThickness = 1.2F;
                        // Avoid per-frame period estimation here – for
                        // connection waveforms a single non-tiled
                        // buffer span is enough.
                        const int periodSamples = kWaveformPoints;

                        const bool tiled = !involvesSampleplay;
                        drawWaveformOnLine(
                            p1, p2, waveformAmplitude,
                            waveformThickness, tempWave,
                            periodSamples,
                            /*normalisation=*/1.0F, 72,
                            /*tiled=*/tiled);
                        drewWave = true;
                    }
                }

                if (!drewWave) {
                    const float thickness = 1.2F;
                    g.drawLine(juce::Line<float>(p1, p2), thickness);
                }
            } else {
                bool drewWave = false;

                if (audioConn) {
                    const float waveformThickness = 1.2F;

                    float tempWave[kWaveformPoints]{};
                    float norm = 0.0F;
                    float rms = 0.0F;

                    const bool hasWave =
                        fetchConnectionWaveformOrAggregate(
                            connKey, conn.from_module_id, tempWave,
                            norm, rms);

                    // Skip waveform rendering for connections with a very
                    // low effective level; a straight line is visually
                    // sufficient and significantly cheaper to rasterise.
                    if (hasWave && rms > 0.0F) {
                        const float amplitudeLevel = juce::jlimit(
                            0.0F, 1.0F, rms / 0.7071F);
                        const float waveformAmplitude =
                            kWaveformAmplitudeScale * amplitudeLevel;
                        // Use the same activeColour (including yellow when
                        // marked for cut) so that all audio-carrying
                        // connections share the same mute/hover visual
                        // behaviour.
                        g.setColour(activeColour);
                        // Avoid per-frame period estimation here – for
                        // connection waveforms a single non-tiled
                        // buffer span is enough.
                        const int periodSamples = kWaveformPoints;
                        const bool tiled = !involvesSampleplay;
                        drawWaveformOnLine(
                            p1, p2, waveformAmplitude,
                            waveformThickness, tempWave,
                            periodSamples,
                            /*normalisation=*/1.0F, 72,
                            /*tiled=*/tiled);
                        drewWave = true;
                    }
                }

                // Dynamic, non-hardlink connection without pulse: draw
                // a straight segment (with or without waveform).
                if (!drewWave) {
                    g.setColour(activeColour);
                    const float thickness = 1.2F;
                    g.drawLine(juce::Line<float>(p1, p2), thickness);
                }
            }

            ++connectionIndex;
        }
    }

    // ---------------------------------------------------------------------
    // Objects: aura + parameter arcs + icon + label.
    // ---------------------------------------------------------------------
    paintObjectsAndPanels(g, bounds, centre, bpmLabelAlpha, nowSeconds);

    // Sequencer widgets anchored to table objects.
    paintSequencerOverlays(g, bounds, centre);

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
        auto getDockBodyColourForObject = [&modules](
                                                       const rectai::ObjectInstance& obj,
                                                       const bool isMuted) {
            juce::Colour activeBase =
                juce::Colour::fromRGB(0x20, 0x90, 0xFF);

            const auto it = modules.find(obj.logical_id());
            if (it != modules.end() && it->second != nullptr) {
                activeBase = colourFromArgb(it->second->colour_argb());
            }

            // Force a fully opaque fill for the node body so that the
            // circular base is always visible even if the underlying
            // ARGB colour coming from the patch carries a low alpha.
            activeBase = activeBase.withAlpha(1.0F);

            // Ensure very dark modules (e.g. black LFO/FX blocks) remain
            // distinguishable against the mostly black background and dock
            // panel by enforcing a minimum brightness for the rendered node
            // body, while preserving hue as much as possible.
            const float brightness = activeBase.getBrightness();
            if (brightness < 0.12F) {
                activeBase = activeBase.brighter(0.5F);
            }

            if (isMuted) {
                return activeBase.darker(1.5F).withMultipliedAlpha(0.8F);
            }

            return activeBase;
        };
        auto dockBounds = bounds;
        const float dockWidth = calculateDockWidth(dockBounds.getWidth());
        juce::Rectangle<float> dockArea =
            dockBounds.removeFromRight(dockWidth);

        // Background panel and static title: use cached image to
        // avoid redrawing the rounded rectangle and text every
        // frame.
        renderDockBackgroundIfNeeded(dockArea.toNearestInt());
        if (!dockBackgroundCache_.isNull()) {
            g.drawImageAt(dockBackgroundCache_,
                          static_cast<int>(dockArea.getX()),
                          static_cast<int>(dockArea.getY()));
        }

        const float titleHeight = 24.0F;
        dockArea.removeFromTop(titleHeight);

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

            for (std::size_t dockIndex = 0; dockIndex < dockedObjects.size(); ++dockIndex) {
                const auto id = dockedObjects[dockIndex].first;
                const auto* obj = dockedObjects[dockIndex].second;

                const float cy = baseY +
                                 (static_cast<float>(dockIndex) + 0.5F) *
                                     slotHeight;
                const float cx = dockArea.getX() +
                                 dockArea.getWidth() * 0.5F;

                // Skip items that are far outside the visible dock area.
                if (cy + nodeRadiusDock < dockArea.getY() ||
                    cy - nodeRadiusDock > dockArea.getBottom()) {
                    continue;
                }

                // Dock capsules reuse the same body colour logic as
                // modules on the table: derive mute from the
                // implicit module -> Output (MASTER_OUTPUT_ID)
                // connection.
                bool isBodyMuted = false;
                for (const auto& conn : scene_.connections()) {
                    if (conn.from_module_id != obj->logical_id() ||
                        conn.to_module_id != rectai::MASTER_OUTPUT_ID) {
                        continue;
                    }

                    const std::string key = makeConnectionKey(conn);
                    if (mutedConnections_.find(key) !=
                        mutedConnections_.end()) {
                        isBodyMuted = true;
                        break;
                    }
                }

                const auto bodyColour =
                    getDockBodyColourForObject(*obj, isBodyMuted);

                g.setColour(bodyColour);
                g.fillEllipse(cx - nodeRadiusDock, cy - nodeRadiusDock,
                              nodeRadiusDock * 2.0F,
                              nodeRadiusDock * 2.0F);

                // Outline in the dock to make the circular capsule
                // clear against the dock background.
                const float outlineThickness = 2.0F;
                const float outlineInset = outlineThickness * 0.5F;
                const juce::Colour outlineColour =
                    (bodyColour.getBrightness() > 0.6F
                         ? juce::Colours::black.withAlpha(0.9F)
                         : juce::Colours::white.withAlpha(0.9F));
                g.setColour(outlineColour);
                g.drawEllipse(cx - nodeRadiusDock + outlineInset,
                              cy - nodeRadiusDock + outlineInset,
                              nodeRadiusDock * 2.0F - outlineThickness,
                              nodeRadiusDock * 2.0F - outlineThickness,
                              outlineThickness);

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
                        // Dynamically tint the icon based on the background
                        // brightness.
                        const float brightness = bodyColour.getBrightness();
                        const juce::Colour iconTint =
                            brightness > 0.6F
                                ? juce::Colour::fromRGB(0x20, 0x20, 0x20)
                                : juce::Colours::white;
                        g.setColour(iconTint.withAlpha(0.9F));

                        const int destX =
                            juce::roundToInt(iconBounds.getX());
                        const int destY =
                            juce::roundToInt(iconBounds.getY());
                        const int destW =
                            juce::roundToInt(iconBounds.getWidth());
                        const int destH =
                            juce::roundToInt(iconBounds.getHeight());

                        auto iconImage =
                            getCachedAtlasIcon(iconId, destW, destH);
                        if (iconImage.isValid()) {
                            g.drawImageAt(iconImage, destX, destY);
                            drewAtlasIcon = true;
                        } else {
                            const auto& src = spriteIt->second.bounds;
                            g.drawImage(atlasImage_, destX, destY, destW,
                                        destH, src.getX(), src.getY(),
                                        src.getWidth(), src.getHeight());
                            drewAtlasIcon = true;
                        }
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

                    // Dynamically tint based on brightness.
                    const float brightness = bodyColour.getBrightness();
                    const juce::Colour iconTint =
                        brightness > 0.6F
                            ? juce::Colour::fromRGB(0x20, 0x20, 0x20)
                            : juce::Colours::white;
                    g.setColour(iconTint.withAlpha(0.9F));

                    const bool isOscillatorIconDock =
                        (iconId == "oscillator" ||
                         iconId.rfind("oscillator_", 0) == 0);

                    if (isOscillatorIconDock) {
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

#if !defined(NDEBUG)
                // Debug overlay in dock: show the module id to the
                // right of the dock bubble to help visual inspection
                // of colours and mapping relative to the ids in the
                // .rtp.
                {
                    const juce::String debugId(obj->logical_id());
                    const float debugMargin = 6.0F;
                    const float debugWidth = 60.0F;
                    const float debugHeight = 14.0F;
                    juce::Rectangle<float> debugBounds(
                        cx + nodeRadiusDock + debugMargin,
                        cy - debugHeight * 0.5F,
                        debugWidth,
                        debugHeight);

                    g.setColour(juce::Colours::white.withAlpha(0.8F));
                    g.setFont(10.0F);
                    g.drawText(debugId, debugBounds,
                               juce::Justification::centredLeft, false);
                }
#endif

                // Tempo controller in dock: mirror the same BPM label
                // used on the main surface so that rotating the docked
                // Tempo tangible still reveals the current session tempo.
                if (moduleForObject != nullptr && bpmLabelAlpha > 0.0) {
                    const auto* tempoModule =
                        dynamic_cast<const rectai::TempoModule*>(
                            moduleForObject);
                    if (tempoModule != nullptr) {
                        const int bpmInt = static_cast<int>(
                            std::round(bpm_));
                        juce::String bpmText(bpmInt);

                        const float margin = 3.0F;
                        juce::Rectangle<float> bpmBounds(
                            cx - nodeRadiusDock,
                            cy - nodeRadiusDock - 14.0F - margin,
                            nodeRadiusDock * 2.0F, 14.0F);

                        const float alpha =
                            static_cast<float>(bpmLabelAlpha);
                        g.setColour(juce::Colours::white.withAlpha(
                            0.9F * alpha));
                        g.setFont(12.0F);
                        g.drawText(bpmText, bpmBounds,
                                   juce::Justification::topLeft, false);
                    }
                }
            }
        }
    }

    // OSC/TUIO indicator HUD on the top-right, left of the dock.
    paintDockAndHud(g, bounds, dockAreaUi, bpmLabelAlpha);

    // Render touch interface visual feedback.
    if (isTouchActive_) {
        bool suppressCursor = false;

        // Hide cursor while dragging modules on the table. For
        // modules being dragged out from the dock, keep showing the
        // cursor only while the pointer remains inside the dock area.
        if (draggedObjectId_ != 0) {
            auto dockBounds = bounds;
            const float dockWidth = calculateDockWidth(dockBounds.getWidth());
            juce::Rectangle<float> dockArea =
                dockBounds.removeFromRight(dockWidth);

            const bool pointerInDock = dockArea.contains(currentTouchPosition_);

            if (!touchStartedInDock_) {
                // Dragging an object that started on the table.
                suppressCursor = true;
            } else if (!pointerInDock) {
                // Dragging a module that started in the dock but the
                // pointer has already left the dock.
                suppressCursor = true;
            }
        }

        // Draw trail only for explicit cut gestures (red cursor) that
        // started in the main window area. Skip trail when holding a
        // connection for mute (white cursor, no trail) or when the
        // cursor is suppressed while dragging modules.
        if (!suppressCursor &&
            isCutModeActive_ &&
            isTouchHeld_ &&
            !touchStartedInDock_ &&
            touchTrail_.size() > 1 &&
            !activeConnectionHold_.has_value()) {
            const double currentTime =
                juce::Time::getMillisecondCounterHiRes() / 1000.0;
            const juce::Colour trailColour = juce::Colour(0xFFFF0000);

            // Draw trail with fade-out effect based on point age.
            for (size_t i = 1; i < touchTrail_.size(); ++i) {
                const auto& prevPoint = touchTrail_[i - 1];
                const auto& currPoint = touchTrail_[i];

                float alpha = 1.0F;
                if (kEnableTrailFade) {
                    const double age = currentTime - currPoint.timestamp;
                    if (age > 0.0 && age < kTrailFadeDurationSeconds) {
                        alpha = static_cast<float>(
                            1.0 - (age / kTrailFadeDurationSeconds));
                    } else if (age >= kTrailFadeDurationSeconds) {
                        alpha = 0.0F;
                    }
                }

                if (alpha > 0.0F) {
                    g.setColour(trailColour.withAlpha(alpha));
                    g.drawLine(prevPoint.position.x, prevPoint.position.y,
                               currPoint.position.x, currPoint.position.y,
                               1.5F);
                }
            }
        }

        if (!suppressCursor) {
            // Draw touch cursor circle (finger representation).
            // White cursor for general interaction (drag modules, sliders,
            // hold-mute). Red cursor only for explicit cut gestures.
            const float touchRadius = 12.0F;
            juce::Colour circleColour;

            if (activeConnectionHold_.has_value()) {
                // Holding a line for temporary mute.
                circleColour = juce::Colours::white.withAlpha(0.8F);
            } else if (touchStartedInDock_) {
                // Interacting with the dock (scroll or docked modules).
                circleColour = juce::Colour(0xFFCCCCCC);
            } else if (isCutModeActive_) {
                // Cut mode inside the musical area.
                circleColour = juce::Colour(0xFFFF0000).withAlpha(0.298F);
            } else {
                // Default interaction mode inside the musical area.
                circleColour = juce::Colours::white.withAlpha(0.8F);
            }

            g.setColour(circleColour);
            g.drawEllipse(currentTouchPosition_.x - touchRadius,
                          currentTouchPosition_.y - touchRadius,
                          touchRadius * 2.0F, touchRadius * 2.0F, 8.0F);
        }
    }
}

void MainComponent::paintCentralPulses(juce::Graphics& g,
                                       const juce::Rectangle<float>& bounds,
                                       juce::Point<float> centre) const
{
    juce::ignoreUnused(bounds);

    // Central node: fixed dot + ripples (already BPM-synchronised).
    // Colour and mute state are driven by the Reactable Output tangible
    // (type=Output) when available.
    const float baseRadius = 6.0F;
    const float masterAlpha = masterMuted_ ? 0.4F : 1.0F;
    g.setColour(masterColour_.withAlpha(masterAlpha));
    g.fillEllipse(centre.x - baseRadius, centre.y - baseRadius,
                  baseRadius * 2.0F, baseRadius * 2.0F);

    for (const auto& pulse : pulses_) {
        const float tRaw = juce::jlimit(0.0F, 1.0F, pulse.age);

        // Ease-out curve for the radial expansion: the pulse grows faster
        // at the beginning and slows down towards the end, using a simple
        // cubic easing equivalent to a Bezier-like acceleration curve.
        const float oneMinus = 1.0F - tRaw;
        const float tRadius = 1.0F - oneMinus * oneMinus * oneMinus;

        const float maxRadius = pulse.strong ? 50.0F : 50.0F;
        const float radius = baseRadius + tRadius * maxRadius;

        // Smooth fade-out curve: keep the alpha stronger for longer and
        // then let it fall off more smoothly near the end of the pulse
        // using a quadratic decay.
        const float fade = oneMinus * oneMinus;
        const float baseAlpha = fade * (masterMuted_ ? 0.4F : 1.0F);
        const float alpha =
            pulse.strong ? baseAlpha * 0.7F : baseAlpha * 0.2F;

        const float baseThickness = pulse.strong ? 4.0F : 2.0F;
        const float thickness = baseThickness + 3.0F * tRadius;

        g.setColour(masterColour_.withAlpha(alpha));
        g.drawEllipse(centre.x - radius, centre.y - radius,
                      radius * 2.0F, radius * 2.0F, thickness);
    }
}

void MainComponent::paintObjectsAndPanels(
    juce::Graphics& g,
    const juce::Rectangle<float>& bounds,
    juce::Point<float> centre,
    const double bpmLabelAlpha,
    const double nowSeconds)
{
    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    const float nodeRadius = 26.0F;

    auto getBodyColourForObject = [&modules](
                                               const rectai::ObjectInstance& obj,
                                               const bool isMuted) {
        juce::Colour activeBase = juce::Colour::fromRGB(0x20, 0x90, 0xFF);

        const auto it = modules.find(obj.logical_id());
        if (it != modules.end() && it->second != nullptr) {
            activeBase = colourFromArgb(it->second->colour_argb());
        }

        // Force a fully opaque fill for the node body so that the
        // circular base is always visible even if the underlying ARGB
        // colour coming from the patch carries a low alpha.
        activeBase = activeBase.withAlpha(1.0F);

        // Ensure very dark modules (e.g. black LFO/FX blocks) remain
        // distinguishable against the mostly black background and dock
        // panel by enforcing a minimum brightness for the rendered node
        // body, while preserving hue as much as possible.
        const float brightness = activeBase.getBrightness();
        if (brightness < 0.12F) {
            activeBase = activeBase.brighter(0.5F);
        }

        if (isMuted) {
            return activeBase.darker(1.5F).withMultipliedAlpha(0.8F);
        }

        return activeBase;
    };

    for (const auto& entry : objects) {
        const auto& object = entry.second;
        if (object.docked() ||
            object.logical_id() == rectai::MASTER_OUTPUT_ID) {
            continue;
        }

        const auto centrePos = objectTableToScreen(object, bounds);
        const auto cx = centrePos.x;
        const auto cy = centrePos.y;

        // A module body is considered muted for colouring purposes
        // when all its effective routes to the master are muted at
        // connection level. This is approximated by checking whether
        // its implicit module -> Output (MASTER_OUTPUT_ID) connection
        // is muted.
        bool isBodyMuted = false;
        for (const auto& conn : scene_.connections()) {
            if (conn.from_module_id != object.logical_id() ||
                conn.to_module_id != rectai::MASTER_OUTPUT_ID) {
                continue;
            }

            const std::string key = makeConnectionKey(conn);
            if (mutedConnections_.find(key) !=
                mutedConnections_.end()) {
                isBodyMuted = true;
                break;
            }
        }

        const juce::Colour bodyColour =
            getBodyColourForObject(object, isBodyMuted);

        // Compute the orientation of the module when it lives in the
        // musical area. Modules outside the table (or docked) keep a
        // fixed screen-space orientation. Inside the music surface
        // they are rotated so that their local "up" axis aligns with
        // the radial audio line (white) connecting them to the master
        // centre.
        const bool insideMusic = isInsideMusicArea(object);
        float rotationAngle = 0.0F;
        if (insideMusic) {
            const float dx = cx - centre.x;
            const float dy = cy - centre.y;
            // Align the module's local "up" axis with the radial
            // audio line. Compensate a 90-degree offset so that the
            // visual upright orientation matches the direction of the
            // white line instead of being rotated to its side.
            rotationAngle =
                std::atan2(dy, dx) -
                juce::MathConstants<float>::halfPi;
        }

        juce::Graphics::ScopedSaveState nodeTransform(g);
        if (insideMusic) {
            g.addTransform(juce::AffineTransform::rotation(rotationAngle,
                                                           cx, cy));
        }

        paintNodeBodyAndOutline(g, cx, cy, nodeRadius, bodyColour);

        // Side controls: left = curved bar (semi-arc) for Freq,
        // right = curved bar (semi-arc) for Gain, both hugging the
        // left/right side of the node. Slightly offset from the
        // node body so that the playhead bar can live between both.
        const float ringRadius = nodeRadius + 10.0F;

        const rectai::AudioModule* moduleForObject = nullptr;
        if (const auto moduleEntryIt = modules.find(object.logical_id());
            moduleEntryIt != modules.end()) {
            moduleForObject = moduleEntryIt->second.get();
        }

        const bool isTempoModule =
            (moduleForObject != nullptr &&
             moduleForObject->is<rectai::TempoModule>());

        float freqValue = 0.5F;
        float gainValue = 0.5F;
        bool showFreqControl = false;
        bool showGainControl = false;
        if (moduleForObject != nullptr) {
            if (isTempoModule) {
                freqValue = rectai::TempoModule::NormalisedFromBpm(bpm_);
            } else if (moduleForObject->is<rectai::LoopModule>()) {
                // For Loop modules, the left bar represents the
                // currently selected sample slot via the normalised
                // "sample" parameter.
                freqValue = moduleForObject->GetParameterOrDefault(
                    "sample", 0.0F);
            } else if (!moduleForObject->uses_pitch_control()) {
                freqValue = moduleForObject->GetParameterOrDefault(
                    "freq",
                    moduleForObject->default_parameter_value("freq"));
            }

            if (const auto* volumeModule =
                    dynamic_cast<const rectai::VolumeModule*>(
                        moduleForObject)) {
                gainValue = volumeModule->GetParameterOrDefault(
                    "volume", 0.9F);
            } else if (moduleForObject->type() ==
                       rectai::ModuleType::kFilter) {
                gainValue = moduleForObject->GetParameterOrDefault(
                    "q",
                    moduleForObject->default_parameter_value("q"));
            } else if (moduleForObject->is<rectai::LoopModule>() ||
                       moduleForObject->is<rectai::SampleplayModule>()) {
                // Loop and Sampleplay modules expose an "amp"
                // parameter that behaves as their primary level
                // control.
                gainValue = moduleForObject->GetParameterOrDefault(
                    "amp", 1.0F);
            } else {
                gainValue = moduleForObject->GetParameterOrDefault(
                    "gain",
                    moduleForObject->default_parameter_value("gain"));
            }

            showFreqControl =
                moduleForObject->uses_frequency_control() ||
                moduleForObject->uses_pitch_control() ||
                isTempoModule ||
                moduleForObject->is<rectai::LoopModule>();
            showGainControl = moduleForObject->uses_gain_control();
        }
        freqValue = juce::jlimit(0.0F, 1.0F, freqValue);
        gainValue = juce::jlimit(0.0F, 1.0F, gainValue);

        // Vertical span of the side arcs: make them almost complete
        // semi-circles while leaving a small gap at the top so the
        // radial audio line (and its waveform) can pass cleanly
        // between both bars. The same margin is used at the bottom to
        // keep the overall shape balanced.
        const float sliderMargin = 3.0F;
        const float sliderTop = cy - ringRadius + sliderMargin;
        const float sliderBottom = cy + ringRadius - sliderMargin;

        paintNodeSideControls(g,
                              object,
                              moduleForObject,
                              cx,
                              cy,
                              ringRadius,
                              sliderTop,
                              sliderBottom,
                              freqValue,
                              gainValue,
                              showFreqControl,
                              showGainControl);

        paintNodeIconAndModeButton(g, object, moduleForObject, cx, cy,
                                   nodeRadius, bodyColour);

        paintNodeModeOverlayAndIcons(g,
                                     object,
                                     moduleForObject,
                                     cx,
                                     cy,
                                     ringRadius,
                                     sliderTop,
                                     sliderBottom,
                                     nodeRadius);

        paintNodeLabelsAndLoopTrail(g,
                                    moduleForObject,
                                    cx,
                                    cy,
                                    nodeRadius,
                                    ringRadius,
                                    bodyColour,
                                    nowSeconds);

#if !defined(NDEBUG)
        paintNodeDebugOverlay(g, object, cx, cy, nodeRadius);
#endif

        // Per-module detail panel (envelope/settings and
        // module-specific tabs) when active.
        paintNodeModulePanel(g, bounds, object, moduleForObject, cx, cy,
                             insideMusic);

        paintNodeTempoOverlay(g,
                              moduleForObject,
                              cx,
                              cy,
                              nodeRadius,
                              bpmLabelAlpha);
    }
}

std::unordered_set<std::int64_t>
MainComponent::computeObjectsWithOutgoingActiveConnection(
    const std::unordered_map<std::string, std::int64_t>& moduleToObjectId) const
{
    std::unordered_set<std::int64_t> result;

    const auto& objects = scene_.objects();

    for (const auto& conn : scene_.connections()) {
        // Ignore connections that target the invisible Output/master module
        // (id MASTER_OUTPUT_ID). Auto-wired master connections should not be
        // treated as "outgoing" for the purpose of hiding generator →
        // master radials.
        if (conn.to_module_id == rectai::MASTER_OUTPUT_ID) {
            continue;
        }

        const auto fromIdIt = moduleToObjectId.find(conn.from_module_id);
        const auto toIdIt = moduleToObjectId.find(conn.to_module_id);
        if (fromIdIt == moduleToObjectId.end() ||
            toIdIt == moduleToObjectId.end()) {
            continue;
        }

        const auto fromObjIt = objects.find(fromIdIt->second);
        const auto toObjIt = objects.find(toIdIt->second);
        if (fromObjIt == objects.end() ||
            toObjIt == objects.end()) {
            continue;
        }

        const auto& fromObj = fromObjIt->second;
        const auto& toObj = toObjIt->second;
        if (!isInsideMusicArea(fromObj) ||
            !isInsideMusicArea(toObj)) {
            continue;
        }

        if (!conn.is_hardlink &&
            !isConnectionGeometricallyActive(fromObj, toObj)) {
            continue;
        }

        result.insert(fromIdIt->second);
    }

    return result;
}

void MainComponent::paintNodeBodyAndOutline(juce::Graphics& g,
                                            float cx,
                                            float cy,
                                            float nodeRadius,
                                            juce::Colour bodyColour)
{
    g.setColour(bodyColour);
    g.fillEllipse(cx - nodeRadius, cy - nodeRadius, nodeRadius * 2.0F,
                  nodeRadius * 2.0F);

    const float outlineThickness = 2.0F;
    const float outlineInset = outlineThickness * 0.5F;
    const juce::Colour outlineColour =
        (bodyColour.getBrightness() > 0.6F
             ? juce::Colours::black.withAlpha(0.9F)
             : juce::Colours::white.withAlpha(0.9F));
    g.setColour(outlineColour);
    g.drawEllipse(cx - nodeRadius + outlineInset,
                  cy - nodeRadius + outlineInset,
                  nodeRadius * 2.0F - outlineThickness,
                  nodeRadius * 2.0F - outlineThickness,
                  outlineThickness);
}

void MainComponent::paintNodeDebugOverlay(
    juce::Graphics& g,
    const rectai::ObjectInstance& object,
    float cx,
    float cy,
    float nodeRadius) const
{
    const juce::String debugId(object.logical_id());
    const float debugMargin = 6.0F;
    const float debugWidth = 60.0F;
    const float debugHeight = 16.0F;
    juce::Rectangle<float> debugBounds(cx + nodeRadius + debugMargin,
                                       cy - debugHeight * 0.5F,
                                       debugWidth,
                                       debugHeight);

    g.setColour(juce::Colours::white.withAlpha(0.8F));
    g.setFont(11.0F);
    g.drawText(debugId, debugBounds,
               juce::Justification::centredLeft, false);
}

void MainComponent::paintNodeTempoOverlay(
    juce::Graphics& g,
    const rectai::AudioModule* moduleForObject,
    float cx,
    float cy,
    float nodeRadius,
    double bpmLabelAlpha) const
{
   if (moduleForObject == nullptr || bpmLabelAlpha <= 0.0) {
       return;
   }

   const auto* tempoModule =
       dynamic_cast<const rectai::TempoModule*>(moduleForObject);
   if (tempoModule == nullptr) {
       return;
   }

   const int bpmInt = static_cast<int>(std::round(bpm_));
   juce::String bpmText(bpmInt);

   const float margin = 4.0F;
   juce::Rectangle<float> bpmBounds(cx - nodeRadius,
                                    cy - nodeRadius - 18.0F - margin,
                                    nodeRadius * 2.0F,
                                    18.0F);

   const float alpha = static_cast<float>(bpmLabelAlpha);
   g.setColour(juce::Colours::white.withAlpha(0.9F * alpha));
   g.setFont(13.0F);
   g.drawText(bpmText, bpmBounds,
              juce::Justification::topLeft,
              false);
}

void MainComponent::paintNodeSideControls(
    juce::Graphics& g,
    const rectai::ObjectInstance& object,
    const rectai::AudioModule* moduleForObject,
    float cx,
    float cy,
    float ringRadius,
    float sliderTop,
    float sliderBottom,
    float freqValue,
    float gainValue,
    bool showFreqControl,
    bool showGainControl)
{
    if (showFreqControl) {
        const bool modeMenuVisibleForThisModule =
            (modeSelection_.menuVisible &&
             modeSelection_.moduleId == object.logical_id());
        const float modeMenuAlpha =
            modeMenuVisibleForThisModule ? 0.15F : 1.0F;
        const juce::Colour freqBackgroundColour =
            juce::Colours::white.withAlpha(0.35F * modeMenuAlpha);
        const juce::Colour freqForegroundColour =
            juce::Colours::white.withAlpha(1.0F * modeMenuAlpha);

        if (moduleForObject != nullptr &&
            moduleForObject->uses_pitch_control()) {
            const float midiNote = moduleForObject->GetParameterOrDefault(
                "midifreq", 57.0F);

            constexpr float kMinMidi = 24.0F;
            constexpr float kMaxMidi = 108.0F;

            float clampedMidi =
                juce::jlimit(kMinMidi, kMaxMidi, midiNote);
            int relative = static_cast<int>(std::floor(
                static_cast<double>(clampedMidi - kMinMidi)));
            if (relative < 0) {
                relative = 0;
            }
            const int octaveIndex = relative / 12;
            const int noteIndex = relative % 12;

            const float dyTop = sliderTop - cy;
            const float dyBottom = sliderBottom - cy;
            const float sinTop = juce::jlimit(-1.0F, 1.0F,
                                              dyTop / ringRadius);
            const float sinBottom = juce::jlimit(-1.0F, 1.0F,
                                                 dyBottom / ringRadius);
            const float angleTop = std::asin(sinTop);
            const float angleBottom = std::asin(sinBottom);

            const int kOctaveSegments = 8;
            const float innerRadius = ringRadius - 5.0F;
            const float octaveGap = 1.0F;

            for (int i = 0; i < kOctaveSegments; ++i) {
                const float segStartT = 1.0F -
                    (static_cast<float>(i + 1) /
                     static_cast<float>(kOctaveSegments));
                const float segEndT = 1.0F -
                    (static_cast<float>(i) /
                     static_cast<float>(kOctaveSegments));

                const float segAngle0 = juce::jmap(
                    segStartT, 0.0F, 1.0F, angleTop, angleBottom);
                const float segAngle1 = juce::jmap(
                    segEndT, 0.0F, 1.0F, angleTop, angleBottom);

                const float shrink = octaveGap / innerRadius;
                const float a0 = segAngle0 + shrink;
                const float a1 = segAngle1 - shrink;
                if (a1 <= a0) {
                    continue;
                }

                juce::Path segPath;
                const int segSteps = 16;
                for (int s = 0; s <= segSteps; ++s) {
                    const float tt = static_cast<float>(s) /
                                     static_cast<float>(segSteps);
                    const float a = juce::jmap(tt, 0.0F, 1.0F,
                                               a0, a1);
                    const float x = cx - innerRadius * std::cos(a);
                    const float y = cy + innerRadius * std::sin(a);
                    if (s == 0) {
                        segPath.startNewSubPath(x, y);
                    } else {
                        segPath.lineTo(x, y);
                    }
                }

                g.setColour(freqBackgroundColour);
                g.strokePath(segPath, juce::PathStrokeType(3.0F));

                if (i == octaveIndex) {
                    g.setColour(freqForegroundColour);
                    g.strokePath(segPath, juce::PathStrokeType(3.0F));
                }
            }

            const int kNoteSegments = 12;
            const float outerRadius = ringRadius + 4.0F;
            const float noteGap = 1.0F;

            float noteCenterAngle = angleTop;

            for (int i = 0; i < kNoteSegments; ++i) {
                const float segStartT = 1.0F -
                    (static_cast<float>(i + 1) /
                     static_cast<float>(kNoteSegments));
                const float segEndT = 1.0F -
                    (static_cast<float>(i) /
                     static_cast<float>(kNoteSegments));

                const float segAngle0 = juce::jmap(
                    segStartT, 0.0F, 1.0F, angleTop, angleBottom);
                const float segAngle1 = juce::jmap(
                    segEndT, 0.0F, 1.0F, angleTop, angleBottom);

                const float shrink = noteGap / outerRadius;
                const float a0 = segAngle0 + shrink;
                const float a1 = segAngle1 - shrink;
                if (a1 <= a0) {
                    continue;
                }

                const float centerAngle = 0.5F * (a0 + a1);
                if (i == noteIndex) {
                    noteCenterAngle = centerAngle;
                }

                juce::Path segPath;
                const int segSteps = 16;
                for (int s = 0; s <= segSteps; ++s) {
                    const float tt = static_cast<float>(s) /
                                     static_cast<float>(segSteps);
                    const float a = juce::jmap(tt, 0.0F, 1.0F,
                                               a0, a1);
                    const float x = cx - outerRadius * std::cos(a);
                    const float y = cy + outerRadius * std::sin(a);
                    if (s == 0) {
                        segPath.startNewSubPath(x, y);
                    } else {
                        segPath.lineTo(x, y);
                    }
                }

                g.setColour(freqBackgroundColour);
                g.strokePath(segPath, juce::PathStrokeType(4.0F));

                if (i == noteIndex) {
                    g.setColour(freqForegroundColour);
                    g.strokePath(segPath, juce::PathStrokeType(4.0F));
                }
            }

            const float triAngle = noteCenterAngle;

            const float tipX = cx - outerRadius * std::cos(triAngle);
            const float tipY = cy + outerRadius * std::sin(triAngle);

            const float ux = (tipX - cx) / outerRadius;
            const float uy = (tipY - cy) / outerRadius;

            const float triHeight = 7.0F;
            const float halfWidth = 4.5F;

            const juce::Point<float> tip(tipX, tipY);
            const juce::Point<float> baseCenter(
                tip.x + ux * triHeight,
                tip.y + uy * triHeight);

            const float tx = -uy;
            const float ty = ux;

            const juce::Point<float> base1(
                baseCenter.x + tx * halfWidth,
                baseCenter.y + ty * halfWidth);
            const juce::Point<float> base2(
                baseCenter.x - tx * halfWidth,
                baseCenter.y - ty * halfWidth);

            juce::Path tri;
            tri.startNewSubPath(tip);
            tri.lineTo(base1);
            tri.lineTo(base2);
            tri.closeSubPath();

            g.setColour(freqForegroundColour);
            g.fillPath(tri);
        } else if (moduleForObject != nullptr &&
                   moduleForObject->is<rectai::LoopModule>()) {
            const int segmentsCount = 4;
            const float gap = 1.0F;

            float sampleParam = juce::jlimit(0.0F, 1.0F, freqValue);
            int activeIndex = static_cast<int>(sampleParam * 4.0F);
            if (activeIndex < 0) {
                activeIndex = 0;
            } else if (activeIndex > 3) {
                activeIndex = 3;
            }

            const float dyTop = sliderTop - cy;
            const float dyBottom = sliderBottom - cy;
            const float sinTop = juce::jlimit(-1.0F, 1.0F,
                                              dyTop / ringRadius);
            const float sinBottom = juce::jlimit(-1.0F, 1.0F,
                                                 dyBottom / ringRadius);
            const float angleTop = std::asin(sinTop);
            const float angleBottom = std::asin(sinBottom);

            for (int i = 0; i < segmentsCount; ++i) {
                const float segStartT = 1.0F -
                    (static_cast<float>(i + 1) /
                     static_cast<float>(segmentsCount));
                const float segEndT = 1.0F -
                    (static_cast<float>(i) /
                     static_cast<float>(segmentsCount));

                const float segAngle0 = juce::jmap(
                    segStartT, 0.0F, 1.0F, angleTop, angleBottom);
                const float segAngle1 = juce::jmap(
                    segEndT, 0.0F, 1.0F, angleTop, angleBottom);

                const float shrink = gap / ringRadius;
                const float a0 = segAngle0 + shrink;
                const float a1 = segAngle1 - shrink;
                if (a1 <= a0) {
                    continue;
                }

                juce::Path segPath;
                const int segSteps = 16;
                for (int s = 0; s <= segSteps; ++s) {
                    const float tt = static_cast<float>(s) /
                                     static_cast<float>(segSteps);
                    const float a = juce::jmap(tt, 0.0F, 1.0F,
                                               a0, a1);
                    const float x = cx - ringRadius * std::cos(a);
                    const float y = cy + ringRadius * std::sin(a);
                    if (s == 0) {
                        segPath.startNewSubPath(x, y);
                    } else {
                        segPath.lineTo(x, y);
                    }
                }

                g.setColour(freqBackgroundColour);
                g.strokePath(segPath, juce::PathStrokeType(5.0F));

                if (i == activeIndex) {
                    g.setColour(freqForegroundColour);
                    g.strokePath(segPath, juce::PathStrokeType(5.0F));
                }
            }

            const float triAngle = juce::jmap(
                sampleParam, 0.0F, 1.0F, angleBottom, angleTop);

            const float barX = cx - ringRadius * std::cos(triAngle);
            const float triY = cy + ringRadius * std::sin(triAngle);

            const float dyTri = triY - cy;
            const float insideTri =
                ringRadius * ringRadius - dyTri * dyTri;
            if (insideTri > 0.0F) {
                const float ux = (barX - cx) / ringRadius;
                const float uy = (triY - cy) / ringRadius;

                const float triHeight = 7.0F;
                const float halfWidth = 4.5F;

                const juce::Point<float> tip(barX, triY);
                const juce::Point<float> baseCenter(
                    tip.x + ux * triHeight,
                    tip.y + uy * triHeight);

                const float tx = -uy;
                const float ty = ux;

                const juce::Point<float> base1(
                    baseCenter.x + tx * halfWidth,
                    baseCenter.y + ty * halfWidth);
                const juce::Point<float> base2(
                    baseCenter.x - tx * halfWidth,
                    baseCenter.y - ty * halfWidth);

                juce::Path tri;
                tri.startNewSubPath(tip);
                tri.lineTo(base1);
                tri.lineTo(base2);
                tri.closeSubPath();

                g.setColour(freqForegroundColour);
                g.fillPath(tri);
            }
        } else {
            juce::Path freqArc;
            const int arcSegments = 40;
            for (int i = 0; i <= arcSegments; ++i) {
                const float t = static_cast<float>(i) /
                                static_cast<float>(arcSegments);
                const float y = juce::jmap(t, 0.0F, 1.0F,
                                           sliderTop, sliderBottom);
                const float dy = y - cy;
                const float inside =
                    ringRadius * ringRadius - dy * dy;
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

            const bool freqIsFull = (freqValue >= 0.999F);
            const bool freqIsEmpty = (freqValue <= 0.001F);

            if (freqIsEmpty) {
                g.setColour(freqBackgroundColour);
                g.strokePath(freqArc, juce::PathStrokeType(5.0F));
            } else {
                const float handleY = juce::jmap(freqValue, 0.0F, 1.0F,
                                                 sliderBottom,
                                                 sliderTop);

                juce::Colour effectiveFreqBackground =
                    freqBackgroundColour;
                juce::Colour effectiveFreqForeground =
                    freqForegroundColour;

                if (freqIsFull) {
                    effectiveFreqBackground = freqForegroundColour;
                }

                g.setColour(effectiveFreqBackground);
                g.strokePath(freqArc, juce::PathStrokeType(5.0F));

                const float handleT = juce::jmap(handleY,
                                                 sliderTop,
                                                 sliderBottom,
                                                 0.0F,
                                                 1.0F);

                juce::Path filledArc;
                const int segments = 64;
                bool started = false;
                for (int i = 0; i <= segments; ++i) {
                    const float t = static_cast<float>(i) /
                                    static_cast<float>(segments);
                    if (t < handleT) {
                        continue;
                    }

                    const float y = juce::jmap(t, 0.0F, 1.0F,
                                               sliderTop,
                                               sliderBottom);
                    const float dy = y - cy;
                    const float inside =
                        ringRadius * ringRadius - dy * dy;
                    if (inside < 0.0F) {
                        continue;
                    }
                    const float dx = std::sqrt(inside);
                    const float x = cx - dx;

                    if (!started) {
                        filledArc.startNewSubPath(x, y);
                        started = true;
                    } else {
                        filledArc.lineTo(x, y);
                    }
                }

                if (!filledArc.isEmpty()) {
                    g.setColour(effectiveFreqForeground);
                    g.strokePath(filledArc,
                                 juce::PathStrokeType(5.0F));
                }
            }
        }
    }

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
        g.strokePath(gainArc, juce::PathStrokeType(1.0F));

        const float handleY = juce::jmap(gainValue, 0.0F, 1.0F,
                                         sliderBottom, sliderTop);
        const float dy = handleY - cy;
        const float inside = ringRadius * ringRadius - dy * dy;
        float handleX = cx + ringRadius;
        if (inside >= 0.0F) {
            const float dx = std::sqrt(inside);
            handleX = cx + dx;
            g.setColour(juce::Colours::white);
            g.fillEllipse(handleX - 4.0F, handleY - 4.0F, 8.0F, 8.0F);
        }

        if (sequencerControlsVolume_ && moduleForObject != nullptr &&
            moduleForObject->is<rectai::OscillatorModule>()) {
            float seqGain = 1.0F;
            const auto it = oscillatorSequencerGain_.find(
                moduleForObject->id());
            if (it != oscillatorSequencerGain_.end()) {
                seqGain = juce::jlimit(0.0F, 1.0F, it->second);
            }

            const float cappedSeq = std::min(gainValue, seqGain);
            if (cappedSeq < gainValue - 0.001F) {
                const float greyY = juce::jmap(cappedSeq, 0.0F, 1.0F,
                                               sliderBottom,
                                               sliderTop);
                const float greyDy = greyY - cy;
                const float greyInside =
                    ringRadius * ringRadius - greyDy * greyDy;
                if (greyInside >= 0.0F) {
                    const float greyDx = std::sqrt(greyInside);
                    const float greyX = cx + greyDx;
                    g.setColour(
                        juce::Colours::lightgrey.withAlpha(0.9F));
                    g.fillEllipse(greyX - 4.0F, greyY - 4.0F,
                                  8.0F, 8.0F);
                }
            }
        }
    }
}

void MainComponent::paintNodeModulePanel(
    juce::Graphics& g,
    const juce::Rectangle<float>& bounds,
    const rectai::ObjectInstance& object,
    const rectai::AudioModule* moduleForObject,
    float cx,
    float cy,
    bool insideMusic)
{
    if (modulePanels_.empty() || !insideMusic) {
        return;
    }

    const auto panelIt = modulePanels_.find(object.logical_id());
    if (panelIt == modulePanels_.end() || !panelIt->second.visible) {
        return;
    }

    const auto& panelState = panelIt->second;
    const auto panelBounds = getModulePanelBounds(object, bounds);

    constexpr float kPanelCornerRadius = 10.0F;
    const juce::Colour panelBg =
        juce::Colour::fromRGB(0x00, 0x10, 0x50);
    const juce::Colour panelBorder =
        juce::Colours::white.withAlpha(0.20F);

    g.setColour(panelBg.withAlpha(0.96F));
    g.fillRoundedRectangle(panelBounds, kPanelCornerRadius);

    g.setColour(panelBorder);
    g.drawRoundedRectangle(panelBounds, kPanelCornerRadius, 2.0F);

    {
        const float triWidth = 12.0F;
        const float triHalfHeight = 7.0F;
        const float leftX = panelBounds.getX();
        const float centreY = cy;

        juce::Path tri;
        tri.startNewSubPath(leftX, centreY - triHalfHeight);
        tri.lineTo(leftX, centreY + triHalfHeight);
        tri.lineTo(leftX - triWidth, centreY);
        tri.closeSubPath();

        g.setColour(panelBorder);
        g.fillPath(tri);
    }

    constexpr float kTabStripHeight = 26.0F;
    juce::Rectangle<float> tabStrip(
        panelBounds.getX(), panelBounds.getBottom(),
        panelBounds.getWidth(), kTabStripHeight);

    const rectai::AudioModule::SettingsTabs* settingsTabsPtr = nullptr;
    if (moduleForObject != nullptr) {
        settingsTabsPtr = &moduleForObject->supported_settings_tabs();
    }

    const int settingsTabCount =
        (settingsTabsPtr != nullptr)
            ? static_cast<int>(settingsTabsPtr->size())
            : 0;
    const int tabCount = settingsTabCount + 1;

    juce::ignoreUnused(tabCount);

    const float tabSide = tabStrip.getHeight();
    const float tabSpacing = 0.0F;

    auto drawTab = [&](int index, const juce::String& iconId,
                       bool isActive) {
        const float x = tabStrip.getX() +
                        (tabSide + tabSpacing) *
                            static_cast<float>(index);
        juce::Rectangle<float> tabBounds(
            x, tabStrip.getY(), tabSide,
            tabStrip.getHeight());

        const auto baseColour =
            isActive ? juce::Colours::white.withAlpha(0.14F)
                     : juce::Colours::white.withAlpha(0.04F);
        g.setColour(baseColour);
        g.fillRect(tabBounds);

        const int destSize = static_cast<int>(
            std::floor(tabBounds.getHeight()));
        if (destSize > 0 && atlasLoaded_) {
            auto iconImage = getCachedAtlasIcon(
                iconId.toStdString(), destSize, destSize);
            if (iconImage.isValid()) {
                const int destX = juce::roundToInt(tabBounds.getX());
                const int destY = juce::roundToInt(tabBounds.getY());
                g.setColour(juce::Colours::white.withAlpha(
                    isActive ? 0.95F : 0.85F));
                g.drawImageAt(iconImage, destX, destY);
            }
        }
    };

    auto mapSettingsKindToPanelTab =
        [](rectai::AudioModule::SettingsTabKind kind) {
            using Kind = rectai::AudioModule::SettingsTabKind;
            switch (kind) {
            case Kind::kEnvelope:
                return ModulePanelState::Tab::kEnvelope;
            case Kind::kLoopFiles:
                return ModulePanelState::Tab::kLoopFiles;
            case Kind::kXYControl:
                return ModulePanelState::Tab::kXYControl;
            case Kind::kSettings:
            default:
                return ModulePanelState::Tab::kSettings;
            }
        };

    for (int i = 0; i < settingsTabCount; ++i) {
        const auto& desc =
            (*settingsTabsPtr)[static_cast<std::size_t>(i)];
        const ModulePanelState::Tab tabKind =
            mapSettingsKindToPanelTab(desc.kind);
        const bool isActive = (panelState.activeTab == tabKind);
        drawTab(i, juce::String(desc.icon_id), isActive);
    }

    drawTab(settingsTabCount, "close_button", false);

    juce::Rectangle<float> contentBounds = panelBounds;
    juce::Rectangle<float> textBounds = contentBounds;
    textBounds.reduce(8.0F, 8.0F);

    g.setColour(juce::Colours::white.withAlpha(0.45F));
    g.setFont(juce::Font(12.0F));

    if (panelState.activeTab == ModulePanelState::Tab::kEnvelope) {
        const auto* envModule =
            dynamic_cast<const rectai::AudioModuleWithEnvelope*>(
                moduleForObject);

        if (envModule != nullptr) {
            auto getIcon = [this](const std::string& iconId,
                                  int width, int height) {
                return getCachedAtlasIcon(iconId, width, height);
            };

            paintModuleEnvelopeView(
                g, contentBounds, *envModule, atlasLoaded_, getIcon,
                kModuleEnvelopeMaxAttackMs,
                kModuleEnvelopeMaxDecayMs,
                kModuleEnvelopeMaxDurationMs,
                kModuleEnvelopeMaxReleaseMs);
        } else {
            g.drawFittedText("Envelope not available",
                              textBounds.toNearestInt(),
                              juce::Justification::centred, 2);
        }
    } else if (panelState.activeTab ==
               ModulePanelState::Tab::kLoopFiles) {
        auto* loopModule = dynamic_cast<rectai::LoopModule*>(
            const_cast<rectai::AudioModule*>(moduleForObject));
        if (loopModule == nullptr) {
            g.drawFittedText("Loop files not available",
                              textBounds.toNearestInt(),
                              juce::Justification::centred, 2);
        } else {
            ensureLoopFileBrowserInitialised(panelState.moduleId,
                                             loopModule);
            rebuildLoopFileBrowserEntries(panelState.moduleId,
                                          loopModule);

            auto* list = getOrCreateLoopFileList(panelState.moduleId);
            if (list != nullptr) {
                juce::Rectangle<float> listBounds =
                    contentBounds.reduced(4.0F, 4.0F);

                juce::Graphics::ScopedSaveState state(g);
                list->setBounds(listBounds.toNearestInt());
                g.reduceClipRegion(listBounds.toNearestInt());
                g.addTransform(juce::AffineTransform::translation(
                    listBounds.getX(), listBounds.getY()));
                list->paint(g);
            }
        }
    } else if (panelState.activeTab ==
               ModulePanelState::Tab::kXYControl) {
        juce::Rectangle<float> xyBounds = contentBounds;
        xyBounds.reduce(6.0F, 6.0F);

        auto* xy = getOrCreateXYControl(panelState.moduleId);
        if (xy == nullptr) {
            g.drawFittedText("XY control not available",
                              textBounds.toNearestInt(),
                              juce::Justification::centred, 2);
        } else {
            const auto* baseModule = moduleForObject;

            xy->setVisualMinimums(0.0F, 0.0F);

            const std::string moduleId = panelState.moduleId;

            const auto mapping = baseModule->xy_control_mapping();
            if (!mapping.has_value()) {
                g.drawFittedText("XY control not available",
                                  textBounds.toNearestInt(),
                                  juce::Justification::centred, 2);
            } else {
                const auto& m = mapping.value();

                const float xValue = baseModule->GetParameterOrDefault(
                    m.x_parameter,
                    baseModule->default_parameter_value(
                        m.x_parameter));
                const float yValue = baseModule->GetParameterOrDefault(
                    m.y_parameter,
                    baseModule->default_parameter_value(
                        m.y_parameter));

                xy->setNormalisedPosition(xValue, yValue, false);

                xy->setOnValueChanged(
                    [this, moduleId, m](float x01, float y01) {
                        scene_.SetModuleParameter(moduleId, m.x_parameter,
                                                  x01);
                        scene_.SetModuleParameter(moduleId, m.y_parameter,
                                                  y01);
                        repaintWithRateLimit();
                    });
            }

            if (xy != nullptr) {
                juce::Graphics::ScopedSaveState state(g);
                xy->setBounds(xyBounds.toNearestInt());
                g.reduceClipRegion(xyBounds.toNearestInt());
                g.addTransform(juce::AffineTransform::translation(
                    xyBounds.getX(), xyBounds.getY()));
                xy->paint(g);
            }
        }
    } else {
        const auto* tempoModule =
            dynamic_cast<const rectai::TempoModule*>(moduleForObject);

        if (tempoModule != nullptr) {
            juce::Rectangle<float> listBounds =
                contentBounds.reduced(4.0F, 4.0F);

            auto* list =
                getOrCreateTempoPresetList(panelState.moduleId);
            if (list != nullptr) {
                const auto& presets =
                    rectai::TempoModule::bpm_presets();
                std::vector<rectai::ui::TextScrollList::Item> items;
                items.reserve(presets.size());

                for (const auto& preset : presets) {
                    rectai::ui::TextScrollList::Item item;
                    item.text = preset.label;
                    items.push_back(std::move(item));
                }

                list->setItems(std::move(items));

                if (!presets.empty()) {
                    const float currentBpm = bpm_;
                    int bestIndex = 0;
                    float bestDiff =
                        std::numeric_limits<float>::max();

                    for (std::size_t i = 0; i < presets.size(); ++i) {
                        const float diff = std::fabs(
                            currentBpm - presets[i].bpm);
                        if (diff < bestDiff) {
                            bestDiff = diff;
                            bestIndex = static_cast<int>(i);
                        }
                    }

                    if (list->getSelectedIndex() != bestIndex) {
                        list->setSelectedIndex(bestIndex, false);
                    }
                }

                juce::Graphics::ScopedSaveState state(g);
                list->setBounds(listBounds.toNearestInt());
                g.reduceClipRegion(listBounds.toNearestInt());
                g.addTransform(juce::AffineTransform::translation(
                    listBounds.getX(), listBounds.getY()));
                list->paint(g);
            }
        } else {
            g.drawFittedText("Module settings coming soon",
                              textBounds.toNearestInt(),
                              juce::Justification::centred, 2);
        }
    }
}

void MainComponent::paintNodeIconAndModeButton(
    juce::Graphics& g,
    const rectai::ObjectInstance& object,
    const rectai::AudioModule* moduleForObject,
    float cx,
    float cy,
    float nodeRadius,
    juce::Colour bodyColour)
{
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

            const float brightness = bodyColour.getBrightness();
            const juce::Colour iconTint =
                brightness > 0.6F
                    ? juce::Colour::fromRGB(0x20, 0x20, 0x20)
                    : juce::Colours::white;
            g.setColour(iconTint.withAlpha(0.9F));

            const int destX = juce::roundToInt(iconBounds.getX());
            const int destY = juce::roundToInt(iconBounds.getY());
            const int destW = juce::roundToInt(iconBounds.getWidth());
            const int destH = juce::roundToInt(iconBounds.getHeight());

            const bool isTempoIcon =
                (moduleForObject != nullptr &&
                 moduleForObject->is<rectai::TempoModule>());
            if (isTempoIcon) {
                juce::Graphics::ScopedSaveState tempoIconState(g);
                g.addTransform(juce::AffineTransform::rotation(
                    -object.angle_radians(), cx, cy));
                g.drawImage(atlasImage_, destX, destY, destW, destH,
                            src.getX(), src.getY(), src.getWidth(),
                            src.getHeight());
            } else {
                g.drawImage(atlasImage_, destX, destY, destW, destH,
                            src.getX(), src.getY(), src.getWidth(),
                            src.getHeight());
            }
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

        const float proceduralBrightness = bodyColour.getBrightness();
        const juce::Colour iconTint =
            proceduralBrightness > 0.6F
                ? juce::Colour::fromRGB(0x20, 0x20, 0x20)
                : juce::Colours::white;
        g.setColour(iconTint.withAlpha(0.9F));

        const bool isOscillatorIcon =
            (iconId == "oscillator" ||
             iconId.rfind("oscillator_", 0) == 0);

        if (isOscillatorIcon) {
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
                const float x = left +
                                (static_cast<float>(i) + 0.5F) *
                                    barWidth;
                const float hFactor = 0.3F +
                                       0.1F * static_cast<float>(i);
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
            juce::String fallbackLabel(object.logical_id());
            if (moduleForObject != nullptr) {
                const auto& moduleLabel = moduleForObject->label();
                if (!moduleLabel.empty()) {
                    fallbackLabel = moduleLabel + " (" +
                                    object.logical_id() + ")";
                }
            }
            const float fallbackBrightness = bodyColour.getBrightness();
            const juce::Colour textTint =
                fallbackBrightness > 0.6F ? juce::Colours::black
                                          : juce::Colours::white;
            g.setColour(textTint);
            g.setFont(14.0F);
            g.drawText(fallbackLabel,
                       juce::Rectangle<float>(cx - nodeRadius,
                                              cy - nodeRadius,
                                              nodeRadius * 2.0F,
                                              nodeRadius * 2.0F),
                       juce::Justification::centred, false);
        }
    }

    const bool hasModeMenu =
        (moduleForObject != nullptr &&
         !moduleForObject->supported_modes().empty());
    if (hasModeMenu) {
        const float buttonWidth = nodeRadius * 1.4F;
        const float buttonHeight = 12.0F;
        const float buttonCenterY = cy + nodeRadius + 8.0F;

        juce::Rectangle<float> buttonBounds(
            cx - buttonWidth * 0.5F,
            buttonCenterY - buttonHeight * 0.5F,
            buttonWidth, buttonHeight);

        const bool isMenuVisible =
            (modeSelection_.menuVisible &&
             modeSelection_.moduleId == object.logical_id());

        std::string modeIconId;
        if (moduleForObject != nullptr) {
            modeIconId = moduleForObject->icon_id();
        }

        if (!modeIconId.empty() && atlasLoaded_ && atlasImage_.isValid()) {
            const int destW = juce::jmin(
                static_cast<int>(buttonBounds.getWidth()) - 4, 18);
            const int destH = destW;
            const int destX = juce::roundToInt(
                buttonBounds.getCentreX() -
                static_cast<float>(destW) * 0.5F);
            const int destY = juce::roundToInt(
                buttonBounds.getCentreY() -
                static_cast<float>(destH) * 0.5F);

            auto iconImage = getCachedAtlasIcon(modeIconId, destW, destH);
            if (iconImage.isValid()) {
                float dragProgress =
                    isMenuVisible ? 1.0F : modeDragProgress_;
                dragProgress = juce::jlimit(0.0F, 1.0F, dragProgress);
                const float iconAlpha = 0.95F - 0.35F * dragProgress;
                g.setColour(juce::Colours::white.withAlpha(iconAlpha));
                g.drawImageAt(iconImage, destX, destY);
            }
        }
    }
}

void MainComponent::paintNodeModeOverlayAndIcons(
    juce::Graphics& g,
    const rectai::ObjectInstance& object,
    const rectai::AudioModule* moduleForObject,
    float cx,
    float cy,
    float ringRadius,
    float sliderTop,
    float sliderBottom,
    float nodeRadius)
{
    const bool drawModeOverlay =
        (modeSelection_.menuVisible &&
         modeSelection_.moduleId == object.logical_id());

    if (!drawModeOverlay || moduleForObject == nullptr) {
        return;
    }

    const auto& modes = moduleForObject->supported_modes();
    const int modeCount = static_cast<int>(modes.size());
    if (modeCount <= 0) {
        return;
    }

    const int maxSlots = 4;
    const float extraLeft = 7.0F;
    const int baseIconSize = 16;

    const float dyTop = sliderTop - cy;
    const float dyBottom = sliderBottom - cy;
    const float sinTop =
        juce::jlimit(-1.0F, 1.0F, dyTop / ringRadius);
    const float sinBottom =
        juce::jlimit(-1.0F, 1.0F, dyBottom / ringRadius);
    float angleTop = std::asin(sinTop);
    const float angleBottom = std::asin(sinBottom);

    {
        const float angleRange = angleTop - angleBottom;
        const float compression = 0.85F;
        angleTop = angleBottom + angleRange * compression;
    }

    const float buttonCenterY = cy + nodeRadius + 8.0F;
    float radiusIcons = ringRadius + 6.0F;
    if (std::abs(sinBottom) > 1.0e-3F) {
        const float candidateRadius =
            (buttonCenterY - cy) / sinBottom;
        if (candidateRadius > 0.0F) {
            radiusIcons = candidateRadius;
        }
    }

    const int slotsToUse = std::min(modeCount, maxSlots);
    juce::ignoreUnused(slotsToUse);

    const int startSlot = 0;
    const int currentIndex = moduleForObject->current_mode_index();

    for (int i = 0; i < modeCount; ++i) {
        const int slotIndex =
            std::min(startSlot + i, maxSlots - 1);
        const float t = static_cast<float>(slotIndex) /
                        static_cast<float>(maxSlots - 1);
        const float angle = juce::jmap(
            t, 0.0F, 1.0F, angleBottom, angleTop);

        const float cosA = std::cos(angle);
        const float sinA = std::sin(angle);

        const float iconCx = cx - radiusIcons * cosA - extraLeft;
        const float iconCy = cy + radiusIcons * sinA;

        const bool isActive = (i == currentIndex);
        const float scale = isActive ? 1.25F : 1.0F;
        const int iconSize = static_cast<int>(
            static_cast<float>(baseIconSize) * scale);
        const int destX = juce::roundToInt(
            iconCx - static_cast<float>(iconSize) * 0.5F);
        const int destY = juce::roundToInt(
            iconCy - static_cast<float>(iconSize) * 0.5F);

        if (!modes[static_cast<std::size_t>(i)].icon_id.empty() &&
            atlasLoaded_ && atlasImage_.isValid()) {
            auto iconImage = getCachedAtlasIcon(
                modes[static_cast<std::size_t>(i)].icon_id,
                iconSize,
                iconSize);
            if (iconImage.isValid()) {
                const float alpha = isActive ? 1.0F : 0.8F;
                g.setColour(
                    juce::Colours::white.withAlpha(alpha));
                g.drawImageAt(iconImage, destX, destY);
            }
        }
    }
}

void MainComponent::paintNodeLabelsAndLoopTrail(
    juce::Graphics& g,
    const rectai::AudioModule* moduleForObject,
    float cx,
    float cy,
    float nodeRadius,
    float ringRadius,
    juce::Colour bodyColour,
    double nowSeconds)
{
    if (const auto* sampleModule =
            dynamic_cast<const rectai::SampleplayModule*>(
                moduleForObject)) {
        const auto* activeInstrument =
            sampleModule->active_instrument();
        if (activeInstrument != nullptr &&
            !activeInstrument->name.empty()) {
            const juce::String instrumentName(activeInstrument->name);
            const float labelMargin = 10.0F;
            const float labelHeight = 18.0F;
            const float labelWidth = 140.0F;
            juce::Rectangle<float> labelBounds(
                cx + nodeRadius + labelMargin,
                cy - labelHeight * 0.5F,
                labelWidth,
                labelHeight);

            double labelAlpha = 0.0;
            const auto itTime =
                sampleplayLabelLastChangeSeconds_.find(
                    sampleModule->id());
            if (itTime != sampleplayLabelLastChangeSeconds_.end()) {
                const double elapsed = nowSeconds - itTime->second;
                if (elapsed >= 0.0) {
                    if (elapsed <= 3.0) {
                        labelAlpha = 1.0;
                    } else if (elapsed <= 3.5) {
                        labelAlpha =
                            1.0 - (elapsed - 3.0) / 0.5;
                    }
                }
            } else {
                labelAlpha = 1.0;
                const double firstNow = nowSeconds;
                sampleplayLabelLastChangeSeconds_.emplace(
                    sampleModule->id(), firstNow);
            }

            if (labelAlpha > 0.0) {
                const float brightness = bodyColour.getBrightness();
                const juce::Colour textColour =
                    brightness > 0.6F ? juce::Colours::white
                                      : juce::Colours::white;
                const float alpha =
                    static_cast<float>(labelAlpha);
                g.setColour(textColour.withAlpha(0.9F * alpha));
                g.setFont(13.0F);
                g.drawText(instrumentName, labelBounds,
                           juce::Justification::centredLeft, false);
            }
        }
    }

    if (const auto* loopModule =
            dynamic_cast<const rectai::LoopModule*>(moduleForObject)) {
        const auto& loops = loopModule->loops();
        if (!loops.empty()) {
            float sampleParam = loopModule->GetParameterOrDefault(
                "sample", 0.0F);
            sampleParam = juce::jlimit(0.0F, 1.0F, sampleParam);
            int selectedIndex =
                static_cast<int>(sampleParam * 4.0F);
            if (selectedIndex < 0) {
                selectedIndex = 0;
            } else if (selectedIndex > 3) {
                selectedIndex = 3;
            }

            const rectai::LoopDefinition* chosen = nullptr;
            if (selectedIndex < static_cast<int>(loops.size())) {
                chosen = &loops[static_cast<std::size_t>(
                    selectedIndex)];
            } else {
                chosen = &loops.front();
            }

            if (chosen != nullptr && !chosen->filename.empty()) {
                juce::String name(chosen->filename);
                const int lastSlash = name.lastIndexOfAnyOf("/\\");
                if (lastSlash >= 0 &&
                    lastSlash < name.length() - 1) {
                    name = name.substring(lastSlash + 1);
                }

                const float labelMargin = 10.0F;
                const float labelHeight = 18.0F;
                const float labelWidth = 160.0F;
                juce::Rectangle<float> labelBounds(
                    cx + nodeRadius + labelMargin,
                    cy - labelHeight * 0.5F,
                    labelWidth,
                    labelHeight);

                double labelAlpha = 0.0;
                const auto itTime =
                    loopLabelLastChangeSeconds_.find(
                        loopModule->id());
                if (itTime != loopLabelLastChangeSeconds_.end()) {
                    const double elapsed =
                        nowSeconds - itTime->second;
                    if (elapsed >= 0.0) {
                        if (elapsed <= 5.0) {
                            labelAlpha = 1.0;
                        } else if (elapsed <= 5.5) {
                            labelAlpha =
                                1.0 - (elapsed - 5.0) / 0.5;
                        }
                    }
                } else {
                    labelAlpha = 1.0;
                    const double firstNow = nowSeconds;
                    loopLabelLastChangeSeconds_.emplace(
                        loopModule->id(), firstNow);
                }

                if (labelAlpha > 0.0) {
                    const juce::Colour textColour =
                        juce::Colours::white;
                    const float alpha =
                        static_cast<float>(labelAlpha);
                    g.setColour(textColour.withAlpha(0.9F * alpha));
                    g.setFont(13.0F);
                    g.drawText(name, labelBounds,
                               juce::Justification::centredLeft,
                               false);
                }
            }
        }

        const auto& loopsForTrail = loopModule->loops();
        if (!loopsForTrail.empty()) {
            float sampleParam = loopModule->GetParameterOrDefault(
                "sample", 0.0F);
            sampleParam = juce::jlimit(0.0F, 1.0F, sampleParam);

            int selectedIndex =
                static_cast<int>(sampleParam * 4.0F);
            if (selectedIndex < 0) {
                selectedIndex = 0;
            } else if (selectedIndex > 3) {
                selectedIndex = 3;
            }

            const rectai::LoopDefinition* chosenTrail = nullptr;
            if (selectedIndex < static_cast<int>(loopsForTrail.size())) {
                chosenTrail = &loopsForTrail[static_cast<std::size_t>(
                    selectedIndex)];
            } else {
                chosenTrail = &loopsForTrail.front();
            }

            if (chosenTrail != nullptr && chosenTrail->beats > 0) {
                const int beatsPerLoop = chosenTrail->beats;
                const double totalBeats =
                    transportBeats_ + beatPhase_;
                const double beatsMod = std::fmod(
                    totalBeats,
                    static_cast<double>(beatsPerLoop));
                const double positiveBeatsMod =
                    (beatsMod < 0.0)
                        ? (beatsMod +
                           static_cast<double>(beatsPerLoop))
                        : beatsMod;
                const double phase01Double =
                    (beatsPerLoop > 0)
                        ? (positiveBeatsMod /
                           static_cast<double>(beatsPerLoop))
                        : 0.0;
                const float phase01 = static_cast<float>(
                    juce::jlimit(0.0, 1.0, phase01Double));

                constexpr double kTrailFadeSeconds = 0.3;
                constexpr int kMaxTrailSegments = 15;

                auto& trail = loopPlayTrails_[loopModule->id()];
                trail.push_back(
                    LoopPlayTrailSample{phase01, nowSeconds});

                const auto isExpired = [nowSeconds](
                                                     const LoopPlayTrailSample& s) {
                    return (nowSeconds - s.timestampSeconds) >
                           kTrailFadeSeconds;
                };

                while (!trail.empty() && isExpired(trail.front())) {
                    trail.erase(trail.begin());
                }

                if (static_cast<int>(trail.size()) >
                    kMaxTrailSegments) {
                    const auto extra = static_cast<int>(
                        trail.size()) - kMaxTrailSegments;
                    trail.erase(trail.begin(),
                                trail.begin() + extra);
                }

                const float innerRadius = nodeRadius + 1.0F;
                const float outerRadius = ringRadius - 4.0F;
                const float barThickness = 2.0F;

                auto drawBarAtPhase = [&](float phase,
                                          const juce::Colour& colour,
                                          float alphaScale) {
                    const float angle =
                        juce::MathConstants<float>::twoPi * phase -
                        juce::MathConstants<float>::halfPi;
                    const float cosA = std::cos(angle);
                    const float sinA = std::sin(angle);

                    const juce::Point<float> p1(
                        cx + innerRadius * cosA,
                        cy + innerRadius * sinA);
                    const juce::Point<float> p2(
                        cx + outerRadius * cosA,
                        cy + outerRadius * sinA);

                    g.setColour(colour.withAlpha(
                        colour.getFloatAlpha() * alphaScale));
                    g.drawLine(juce::Line<float>(p1, p2),
                               barThickness);
                };

                for (const auto& sample : trail) {
                    const double age =
                        nowSeconds - sample.timestampSeconds;
                    if (age < 0.0 || age > kTrailFadeSeconds) {
                        continue;
                    }

                    const float t = static_cast<float>(
                        age / kTrailFadeSeconds);
                    const float alpha =
                        juce::jlimit(0.0F, 1.0F, 1.0F - t);
                    if (alpha <= 0.0F) {
                        continue;
                    }

                    drawBarAtPhase(sample.phase01,
                                   juce::Colours::white,
                                   alpha * 0.8F);
                }

                drawBarAtPhase(phase01, juce::Colours::red, 1.0F);
            }
        }
    }
}

void MainComponent::paintSequencerOverlays(
    juce::Graphics& g,
    const juce::Rectangle<float>& bounds,
    juce::Point<float> centre) const
{
    const auto& objects = scene_.objects();

    for (const auto& entry : objects) {
        const auto& object = entry.second;
        if (object.docked() ||
            object.logical_id() == rectai::MASTER_OUTPUT_ID) {
            continue;
        }

        const auto centrePos = objectTableToScreen(object, bounds);
        const auto cx = centrePos.x;
        const auto cy = centrePos.y;

        // Simple sequencer: attach to any object whose logical_id starts
        // with "seq" for now. When the object is in the musical area
        // the sequencer grid orientation follows the same radial
        // direction as the audio line so that it visually projects
        // outwards from the table centre.
        if (object.logical_id().rfind("seq", 0) == 0) {
            const int cols = 8;
            const int rows = 3;
            const float cellSize = 14.0F;
            const float gridWidth = cols * cellSize;
            const float gridHeight = rows * cellSize;

            float angle = object.angle_radians();
            if (isInsideMusicArea(object)) {
                const float dx = cx - centre.x;
                const float dy = cy - centre.y;
                angle = std::atan2(dy, dx) -
                        juce::MathConstants<float>::halfPi;
            }

            const float nodeRadius = 26.0F;
            const float offset = nodeRadius + 40.0F;
            const float gx = cx + offset * std::cos(angle);
            const float gy = cy + offset * std::sin(angle);

            juce::Rectangle<float> gridBounds(
                gx - gridWidth * 0.5F, gy - gridHeight * 0.5F,
                gridWidth, gridHeight);

            g.setColour(juce::Colours::white.withAlpha(0.2F));
            g.drawRoundedRectangle(gridBounds, 4.0F, 1.0F);

            // Draw grid points.
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {
                    const float x = gridBounds.getX() +
                                    (c + 0.5F) * cellSize;
                    const float y = gridBounds.getY() +
                                    (r + 0.5F) * cellSize;

                    const bool isActiveStep = (c == sequencerStep_);
                    const float alpha = isActiveStep ? 0.9F : 0.3F;
                    const float radius = isActiveStep ? 3.5F : 2.0F;

                    g.setColour(
                        juce::Colours::white.withAlpha(alpha));
                    g.fillEllipse(x - radius, y - radius,
                                  radius * 2.0F, radius * 2.0F);
                }
            }
        }
    }
}

void MainComponent::paintDockAndHud(
    juce::Graphics& g,
    const juce::Rectangle<float>& bounds,
    const juce::Rectangle<float>& dockAreaUi,
    const double bpmLabelAlpha)
{
    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    // Dock bar: stacked modules that are marked as docked in the .rtp.
    // These modules live outside the main musical surface but remain
    // visible and accessible in a dedicated side strip.
    std::vector<std::pair<std::int64_t, const rectai::ObjectInstance*>>
        dockedObjects;
    dockedObjects.reserve(objects.size());
    for (const auto& [id, obj] : objects) {
        if (obj.docked()) {
            dockedObjects.emplace_back(id, &obj);
        }
    }

    juce::Rectangle<float> dockArea = dockAreaUi;

    if (!dockedObjects.empty()) {
        auto getDockBodyColourForObject = [&modules](
                                                       const rectai::ObjectInstance& obj,
                                                       const bool isMuted) {
            juce::Colour activeBase =
                juce::Colour::fromRGB(0x20, 0x90, 0xFF);

            const auto it = modules.find(obj.logical_id());
            if (it != modules.end() && it->second != nullptr) {
                activeBase = colourFromArgb(it->second->colour_argb());
            }

            // Force a fully opaque fill for the node body so that the
            // circular base is always visible even if the underlying
            // ARGB colour coming from the patch carries a low alpha.
            activeBase = activeBase.withAlpha(1.0F);

            // Ensure very dark modules (e.g. black LFO/FX blocks) remain
            // distinguishable against the mostly black background and dock
            // panel by enforcing a minimum brightness for the rendered node
            // body, while preserving hue as much as possible.
            const float brightness = activeBase.getBrightness();
            if (brightness < 0.12F) {
                activeBase = activeBase.brighter(0.5F);
            }

            if (isMuted) {
                return activeBase.darker(1.5F).withMultipliedAlpha(0.8F);
            }

            return activeBase;
        };

        // Background panel and static title: use cached image to
        // avoid redrawing the rounded rectangle and text every
        // frame.
        renderDockBackgroundIfNeeded(dockArea.toNearestInt());
        if (!dockBackgroundCache_.isNull()) {
            g.drawImageAt(dockBackgroundCache_,
                          static_cast<int>(dockArea.getX()),
                          static_cast<int>(dockArea.getY()));
        }

        const float titleHeight = 24.0F;
        dockArea.removeFromTop(titleHeight);

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

            for (std::size_t dockIndex = 0;
                 dockIndex < dockedObjects.size(); ++dockIndex) {
                const auto* obj = dockedObjects[dockIndex].second;

                const float cy = baseY +
                                 (static_cast<float>(dockIndex) + 0.5F) *
                                     slotHeight;
                const float cx = dockArea.getX() +
                                 dockArea.getWidth() * 0.5F;

                // Skip items that are far outside the visible dock area.
                if (cy + nodeRadiusDock < dockArea.getY() ||
                    cy - nodeRadiusDock > dockArea.getBottom()) {
                    continue;
                }

                // Dock capsules reuse the same body colour logic as
                // modules on the table: derive mute from the
                // implicit module -> Output (MASTER_OUTPUT_ID)
                // connection.
                bool isBodyMuted = false;
                for (const auto& conn : scene_.connections()) {
                    if (conn.from_module_id != obj->logical_id() ||
                        conn.to_module_id != rectai::MASTER_OUTPUT_ID) {
                        continue;
                    }

                    const std::string key = makeConnectionKey(conn);
                    if (mutedConnections_.find(key) !=
                        mutedConnections_.end()) {
                        isBodyMuted = true;
                        break;
                    }
                }

                const auto bodyColour =
                    getDockBodyColourForObject(*obj, isBodyMuted);

                g.setColour(bodyColour);
                g.fillEllipse(cx - nodeRadiusDock, cy - nodeRadiusDock,
                              nodeRadiusDock * 2.0F,
                              nodeRadiusDock * 2.0F);

                // Outline in the dock to make the circular capsule
                // clear against the dock background.
                const float outlineThickness = 2.0F;
                const float outlineInset = outlineThickness * 0.5F;
                const juce::Colour outlineColour =
                    (bodyColour.getBrightness() > 0.6F
                         ? juce::Colours::black.withAlpha(0.9F)
                         : juce::Colours::white.withAlpha(0.9F));
                g.setColour(outlineColour);
                g.drawEllipse(cx - nodeRadiusDock + outlineInset,
                              cy - nodeRadiusDock + outlineInset,
                              nodeRadiusDock * 2.0F - outlineThickness,
                              nodeRadiusDock * 2.0F - outlineThickness,
                              outlineThickness);

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
                        // Dynamically tint the icon based on the background
                        // brightness.
                        const float brightness = bodyColour.getBrightness();
                        const juce::Colour iconTint =
                            brightness > 0.6F
                                ? juce::Colour::fromRGB(0x20, 0x20, 0x20)
                                : juce::Colours::white;
                        g.setColour(iconTint.withAlpha(0.9F));

                        const int destX =
                            juce::roundToInt(iconBounds.getX());
                        const int destY =
                            juce::roundToInt(iconBounds.getY());
                        const int destW =
                            juce::roundToInt(iconBounds.getWidth());
                        const int destH =
                            juce::roundToInt(iconBounds.getHeight());

                        auto iconImage =
                            getCachedAtlasIcon(iconId, destW, destH);
                        if (iconImage.isValid()) {
                            g.drawImageAt(iconImage, destX, destY);
                            drewAtlasIcon = true;
                        } else {
                            const auto& src = spriteIt->second.bounds;
                            g.drawImage(atlasImage_, destX, destY, destW,
                                        destH, src.getX(), src.getY(),
                                        src.getWidth(), src.getHeight());
                            drewAtlasIcon = true;
                        }
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

                    // Dynamically tint based on brightness.
                    const float brightness = bodyColour.getBrightness();
                    const juce::Colour iconTint =
                        brightness > 0.6F
                            ? juce::Colour::fromRGB(0x20, 0x20, 0x20)
                            : juce::Colours::white;
                    g.setColour(iconTint.withAlpha(0.9F));

                    const bool isOscillatorIconDock =
                        (iconId == "oscillator" ||
                         iconId.rfind("oscillator_", 0) == 0);

                    if (isOscillatorIconDock) {
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

#if !defined(NDEBUG)
                // Debug overlay in dock: show the module id to the
                // right of the dock bubble to help visual inspection
                // of colours and mapping relative to the ids in the
                // .rtp.
                {
                    const juce::String debugId(obj->logical_id());
                    const float debugMargin = 6.0F;
                    const float debugWidth = 60.0F;
                    const float debugHeight = 14.0F;
                    juce::Rectangle<float> debugBounds(
                        cx + nodeRadiusDock + debugMargin,
                        cy - debugHeight * 0.5F,
                        debugWidth,
                        debugHeight);

                    g.setColour(juce::Colours::white.withAlpha(0.8F));
                    g.setFont(10.0F);
                    g.drawText(debugId, debugBounds,
                               juce::Justification::centredLeft, false);
                }
#endif

                // Tempo controller in dock: mirror the same BPM label
                // used on the main surface so that rotating the docked
                // Tempo tangible still reveals the current session tempo.
                if (moduleForObject != nullptr && bpmLabelAlpha > 0.0) {
                    const auto* tempoModule =
                        dynamic_cast<const rectai::TempoModule*>(
                            moduleForObject);
                    if (tempoModule != nullptr) {
                        const int bpmInt = static_cast<int>(
                            std::round(bpm_));
                        juce::String bpmText(bpmInt);

                        const float margin = 3.0F;
                        juce::Rectangle<float> bpmBounds(
                            cx - nodeRadiusDock,
                            cy - nodeRadiusDock - 14.0F - margin,
                            nodeRadiusDock * 2.0F, 14.0F);

                        const float alpha =
                            static_cast<float>(bpmLabelAlpha);
                        g.setColour(juce::Colours::white.withAlpha(
                            0.9F * alpha));
                        g.setFont(12.0F);
                        g.drawText(bpmText, bpmBounds,
                                   juce::Justification::topLeft, false);
                    }
                }
            }
        }
    }

    // OSC/TUIO traffic indicator label (top-right, left of the dock).
    {
        const double nowSeconds =
            juce::Time::getMillisecondCounterHiRes() / 1000.0;

        bool showLabel = false;
        bool showPulse = false;
        bool isTuio = false;

        if (lastInputActivitySeconds_ > 0.0) {
            const double idle = nowSeconds - lastInputActivitySeconds_;
            if (idle >= 0.0 && idle <= 60.0) {
                showLabel = true;
                isTuio = (lastInputActivityKind_ ==
                          InputActivityKind::kTuio);
            }
        }

        if (inputActivityPulseSeconds_ > 0.0) {
            const double pulseAge = nowSeconds - inputActivityPulseSeconds_;
            if (pulseAge >= 0.0 && pulseAge <= 0.25) {
                showPulse = true;
            }
        }

        if (showLabel) {
            juce::Graphics::ScopedSaveState indicatorState(g);

            const juce::String labelText =
                isTuio ? juce::String("TUIO") : juce::String("OSC");

            const float margin = 8.0F;
            const float height = 20.0F;
            const float paddingH = 8.0F;
            const float paddingV = 4.0F;
            const float bubbleDiameter = 8.0F;

            juce::Font font(13.0F, juce::Font::plain);
            g.setFont(font);
            const float textWidth =
                g.getCurrentFont().getStringWidthFloat(labelText);

            const float extraForPulse =
                showPulse ? (bubbleDiameter + 6.0F) : 0.0F;
            const float totalWidth =
                textWidth + paddingH * 2.0F + extraForPulse;

            const float rightX = dockArea.getX() - margin;
            const float topY = bounds.getY() + margin;

            juce::Rectangle<float> bg(rightX - totalWidth, topY,
                                      totalWidth, height);

            g.setColour(juce::Colours::black.withAlpha(0.6F));
            g.fillRoundedRectangle(bg, 6.0F);

            g.setColour(juce::Colours::white.withAlpha(0.85F));
            juce::Rectangle<float> textBounds =
                bg.reduced(paddingH, paddingV);
            if (showPulse) {
                textBounds.setRight(textBounds.getRight() -
                                    (bubbleDiameter + 6.0F));
            }
            g.drawText(labelText, textBounds,
                       juce::Justification::centredLeft, false);

            if (showPulse) {
                const float cx = bg.getRight() - paddingH -
                                 bubbleDiameter * 0.5F;
                const float cy = bg.getCentreY();
                const float radius = bubbleDiameter * 0.5F;

                g.setColour(juce::Colours::green.withAlpha(0.9F));
                g.fillEllipse(cx - radius, cy - radius,
                              bubbleDiameter, bubbleDiameter);
            }
        }
    }
}
