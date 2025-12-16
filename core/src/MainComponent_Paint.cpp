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
    g.fillAll(juce::Colours::black);

    const auto bounds = getLocalBounds().toFloat();
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

    // ---------------------------------------------------------------------
    // Background: solid table colour (#001a80) with a soft outer
    // border that fades to black so the edge blends into the
    // surrounding black background. When the audio device could not
    // be initialised due to having no output channels, the table is
    // rendered in red (#801a1a) to signal the degraded audio state.
    // ---------------------------------------------------------------------
    const float tableRadius =
        0.45F * std::min(bounds.getWidth(), bounds.getHeight());

    const bool hasAudioInitError =
        audioEngine_.hasInitialisationError();

    const juce::Colour tableColour = hasAudioInitError
                                         ? juce::Colour::fromRGB(
                                               0x80, 0x1a, 0x1a)  // #801a1a
                                         : juce::Colour::fromRGB(
                                               0x00, 0x1a, 0x80);  // #001a80

    // Draw a subtle outer ring that transitions from the table
    // colour to black, creating a smooth fade with the background.
    const float borderThickness = 40.0F;
    const float outerRadius = tableRadius + borderThickness;

    {
        juce::Graphics::ScopedSaveState borderState(g);

        juce::Path borderRing;
        borderRing.addEllipse(centre.x - outerRadius,
                              centre.y - outerRadius,
                              outerRadius * 2.0F,
                              outerRadius * 2.0F);
        borderRing.addEllipse(centre.x - tableRadius,
                              centre.y - tableRadius,
                              tableRadius * 2.0F,
                              tableRadius * 2.0F);
        borderRing.setUsingNonZeroWinding(false);  // even-odd: ring only.

        juce::ColourGradient borderGradient(tableColour, centre.x, centre.y,
                                            juce::Colours::black, centre.x,
                                            centre.y + outerRadius, true);

        // Ensure the colour at the table edge remains the solid
        // table colour while the outermost edge fades to black.
        const double innerStop =
            static_cast<double>(tableRadius / outerRadius);
        borderGradient.addColour(innerStop, tableColour);
        borderGradient.addColour(1.0, juce::Colours::black);

        g.setGradientFill(borderGradient);
        g.fillPath(borderRing);
    }

    // Solid table disc without any internal gradient.
    g.setColour(tableColour);
    g.fillEllipse(centre.x - tableRadius, centre.y - tableRadius,
                  tableRadius * 2.0F, tableRadius * 2.0F);

    const auto& objects = scene_.objects();
    const auto& modules = scene_.modules();

    // Fetch recent per-voice mono snapshots from the audio engine so
    // that audio-carrying lines can display waveforms that better
    // match the module or chain that is actually producing sound.
    constexpr int kWaveformPoints = 512;
    // Pre-filter (raw oscillator) and post-filter (processed) per-voice
    // snapshots so that generator→module connections can show the
    // original waveform while filters/FX and master visuals reflect
    // the processed audio.
    float voiceWaveformsPre[AudioEngine::kMaxVoices]
                           [kWaveformPoints]{};
    float voiceWaveformsPost[AudioEngine::kMaxVoices]
                            [kWaveformPoints]{};
    float voiceNormPre[AudioEngine::kMaxVoices]{};
    float voiceNormPost[AudioEngine::kMaxVoices]{};
    float voiceRmsPost[AudioEngine::kMaxVoices]{};

    // For visual consistency (especially with saw waves), we estimate
    // an approximate period for each voice in the snapshot so that the
    // UI can repeat a single cycle of the waveform along long lines
    // instead of traversing an arbitrary window of history.
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

    int voicePeriodSamples[AudioEngine::kMaxVoices]{};
    for (int v = 0; v < AudioEngine::kMaxVoices; ++v) {
        audioEngine_.getVoiceWaveformSnapshot(
            v, voiceWaveformsPre[v], kWaveformPoints, 0.05);
        audioEngine_.getVoiceFilteredWaveformSnapshot(
            v, voiceWaveformsPost[v], kWaveformPoints, 0.05);

        float maxAbsPre = 0.0F;
        float maxAbsPost = 0.0F;
        float sumSquaresPost = 0.0F;
        for (int i = 0; i < kWaveformPoints; ++i) {
            const float sPre = voiceWaveformsPre[v][i];
            const float sPost = voiceWaveformsPost[v][i];
            maxAbsPre = std::max(maxAbsPre, std::abs(sPre));
            maxAbsPost = std::max(maxAbsPost, std::abs(sPost));
            sumSquaresPost += sPost * sPost;
        }

        voiceNormPre[v] =
            maxAbsPre > 1.0e-4F ? 1.0F / maxAbsPre : 0.0F;
        voiceNormPost[v] =
            maxAbsPost > 1.0e-4F ? 1.0F / maxAbsPost : 0.0F;

        if (kWaveformPoints > 0) {
            const float meanSquarePost = sumSquaresPost /
                                         static_cast<float>(
                                             kWaveformPoints);
            voiceRmsPost[v] = meanSquarePost > 0.0F
                                  ? std::sqrt(meanSquarePost)
                                  : 0.0F;
        } else {
            voiceRmsPost[v] = 0.0F;
        }

        if (voiceNormPre[v] > 0.0F) {
            voicePeriodSamples[v] =
                estimateWaveformPeriod(voiceWaveformsPre[v],
                                       kWaveformPoints);
        } else {
            voicePeriodSamples[v] = 0;
        }
    }

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

    std::unordered_map<std::string, std::int64_t> moduleToObjectId;
    for (const auto& [id, object] : objects) {
        moduleToObjectId.emplace(object.logical_id(), id);
    }

    std::unordered_set<std::int64_t> objectsWithOutgoingActiveConnection;
    for (const auto& conn : scene_.connections()) {
        // Ignore connections that target the invisible Output/master module
        // (id MASTER_OUTPUT_ID). Auto-wired master connections should not be
        // treated as "outgoing" for the purpose of hiding generator →
        // master lines.
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
            const float maxRadius = pulse.strong ? 50.0F : 50.0F;
            const float radius = baseRadius + t * maxRadius;

            const float baseAlpha =
                (1.0F - t) * (masterMuted_ ? 0.4F : 1.0F);
            const float alpha =
                pulse.strong ? baseAlpha * 0.7F : baseAlpha * 0.2F;

            const float thickness = pulse.strong ? 4.0F : 2.0F;

            g.setColour(masterColour_.withAlpha(alpha));
            g.drawEllipse(centre.x - radius, centre.y - radius, radius * 2.0F,
                          radius * 2.0F, thickness);
        }

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
                // whether their radial line carries sound.
                isSampleplayModule ||
                modulesWithActiveAudio_.find(object.logical_id()) !=
                    modulesWithActiveAudio_.end();

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
                
                // First segment: object to split (with waveform even if muted).
                // Check for voice data directly, not relying on modulesWithActiveAudio_
                // since the module is temporarily muted during hold.
                const auto voiceIt = moduleVoiceIndex_.find(object.logical_id());
                const int voiceIndex = (voiceIt != moduleVoiceIndex_.end()) ? voiceIt->second : -1;

                if (voiceIndex >= 0 && voiceIndex < AudioEngine::kMaxVoices &&
                    voiceNormPost[voiceIndex] > 0.0F) {
                    g.setColour(juce::Colours::white.withAlpha(baseAlpha));

                    float amplitudeLevel = visualLevel;
                    if (!hasExplicitVolumeBar) {
                        const float rms = voiceRmsPost[voiceIndex];
                        if (rms > 0.0F) {
                            amplitudeLevel = juce::jlimit(0.0F, 1.0F,
                                                          rms / 0.7071F);
                        } else {
                            amplitudeLevel = 0.0F;
                        }
                    }

                    const float waveformAmplitude = 15.0F * amplitudeLevel;
                    const float waveformThickness = 1.4F;
                    // Calculate segments based on physical line length in pixels to maintain
                    // consistent waveform visual density (window effect: longer line shows more cycles).
                    const juce::Point<float> delta =
                        splitPoint - juce::Point<float>{cx, cy};
                    const float segmentLineLength =
                        std::sqrt(delta.x * delta.x +
                                  delta.y * delta.y);
                    const int segmentsForWaveformHold =
                        static_cast<int>(segmentLineLength / 5.0F);
                    const int periodSamples =
                        voicePeriodSamples[voiceIndex] > 0
                            ? voicePeriodSamples[voiceIndex]
                            : kWaveformPoints;
                    drawWaveformOnLine(
                        {cx, cy}, splitPoint, waveformAmplitude,
                        waveformThickness,
                        voiceWaveformsPost[voiceIndex], periodSamples,
                        voiceNormPost[voiceIndex],
                        juce::jmax(1, segmentsForWaveformHold),
                        /*tiled=*/true);
                } else {
                    g.setColour(juce::Colours::white.withAlpha(baseAlpha));
                    g.drawLine(juce::Line<float>({cx, cy}, splitPoint), 1.2F);
                }
                
                // Second segment: split to center (dashed, muted/silenced output).
                const float dashLengths[] = {6.0F, 4.0F};
                g.setColour(juce::Colours::white.withAlpha(baseAlpha * 0.6F));
                g.drawDashedLine(juce::Line<float>(splitPoint, centre), dashLengths,
                                 static_cast<int>(std::size(dashLengths)), 1.2F);
                continue;
            }

            // A radial line is visually muted when its implicit
            // connection to the master Output (MASTER_OUTPUT_ID) is
            // muted at the connection level. We derive that state by
            // checking for a module -> MASTER_OUTPUT_ID connection in
            // mutedConnections_.
            bool isRadialMuted = false;
            {
                for (const auto& conn : scene_.connections()) {
                    if (conn.from_module_id != object.logical_id() ||
                        conn.to_module_id != rectai::MASTER_OUTPUT_ID) {
                        continue;
                    }

                    const std::string key = makeConnectionKey(conn);
                    if (mutedConnections_.find(key) !=
                        mutedConnections_.end()) {
                        isRadialMuted = true;
                        break;
                    }
                }
            }

            // Detect if this module (typically a Filter) is being
            // driven directly by a Sampleplay module via an active
            // audio connection. In that case, we can reuse the
            // Sampleplay waveform for its radial even though it does
            // not occupy a generator voice.
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
                const auto radialColour =
                    isMarkedForCut
                        ? juce::Colours::yellow.withAlpha(0.9F)
                        : juce::Colours::white.withAlpha(baseAlpha);
                g.setColour(radialColour);
                const float amplitudeLevel =
                    isSampleplayModule ? visualLevel : 1.0F;
                const float waveformAmplitude =
                    15.0F * amplitudeLevel;
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
                const auto radialColour =
                    isMarkedForCut
                        ? juce::Colours::yellow.withAlpha(0.9F)
                        : juce::Colours::white.withAlpha(baseAlpha);
                g.setColour(radialColour);
                const float amplitudeLevel = visualLevel;
                const float waveformAmplitude =
                    15.0F * amplitudeLevel;
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

            if (lineCarriesAudio && !isRadialMuted) {
                const auto voiceIt =
                    moduleVoiceIndex_.find(object.logical_id());
                const int voiceIndex =
                    (voiceIt != moduleVoiceIndex_.end())
                        ? voiceIt->second
                        : -1;

                if (voiceIndex >= 0 &&
                    voiceIndex < AudioEngine::kMaxVoices &&
                    voiceNormPost[voiceIndex] > 0.0F) {
                    const auto waveformColour =
                        isMarkedForCut
                            ? juce::Colours::yellow.withAlpha(0.9F)
                            : juce::Colours::white.withAlpha(baseAlpha);
                    g.setColour(waveformColour);
                    float amplitudeLevel = visualLevel;

                    // If this module does not have an explicit
                    // volume bar, derive the visual level from the
                    // RMS of the corresponding audio voice so that
                    // filters/FX reflect their actual output level
                    // instead of a fixed height.
                    if (!hasExplicitVolumeBar) {
                        const float rms =
                            voiceRmsPost[voiceIndex];
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
                        15.0F * amplitudeLevel;
                    const float waveformThickness = 1.4F;
                    const int periodSamples =
                        voicePeriodSamples[voiceIndex] > 0
                            ? voicePeriodSamples[voiceIndex]
                            : kWaveformPoints;
                    drawWaveformOnLine(
                        line.getEnd(), line.getStart(),
                        waveformAmplitude, waveformThickness,
                        voiceWaveformsPost[voiceIndex], periodSamples,
                        voiceNormPost[voiceIndex], segmentsForWaveform,
                        /*tiled=*/true);
                    continue;
                }
            }

            // Fallback for non-audio or silent lines. Preserve the
            // existing style (solid vs dashed) and only change the
            // colour when the line is marked for a pending
            // mute/unmute operation.
            const auto radialFallbackColour =
                isMarkedForCut
                    ? juce::Colours::yellow.withAlpha(0.9F)
                    : juce::Colours::white.withAlpha(baseAlpha);
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
                (modulesWithActiveAudio_.find(conn.from_module_id) !=
                     modulesWithActiveAudio_.end() ||
                 modulesWithActiveAudio_.find(conn.to_module_id) !=
                     modulesWithActiveAudio_.end());

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
              const float connectionLevel = juce::jmax(fromLevel, toLevel);

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

                audioEngine_.getConnectionWaveformSnapshot(
                    connKey, tempWave, kWaveformPoints, 0.05);

                float maxAbs = 0.0F;
                for (int i = 0; i < kWaveformPoints; ++i) {
                    maxAbs = std::max(maxAbs, std::abs(tempWave[i]));
                }

                if (maxAbs > 1.0e-4F) {
                    norm = 1.0F / maxAbs;
                }

                bool drewWave = false;
                if (norm > 0.0F) {
                    g.setColour(activeColour);
                    const float waveformAmplitude =
                        (conn.is_hardlink ? 10.0F : 10.0F) * connectionLevel;
                    const float waveformThickness = conn.is_hardlink ? 1.2F : 1.2F;
                    const juce::Point<float> delta = splitPoint - p1;
                    const float lineLength = std::sqrt(
                        delta.x * delta.x + delta.y * delta.y);
                    const int segmentsForWaveform =
                        static_cast<int>(lineLength / 5.0F);
                    const int periodSamples =
                        estimateWaveformPeriod(tempWave,
                                               kWaveformPoints);

                        const bool tiled = !involvesSampleplay;
                        drawWaveformOnLine(
                            p1, splitPoint, waveformAmplitude,
                        waveformThickness, tempWave,
                            periodSamples, norm,
                            juce::jmax(1, segmentsForWaveform),
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

                    audioEngine_.getConnectionWaveformSnapshot(
                        connKey, tempWave, kWaveformPoints, 0.05);

                    float maxAbs = 0.0F;
                    for (int i = 0; i < kWaveformPoints; ++i) {
                        maxAbs = std::max(maxAbs, std::abs(tempWave[i]));
                    }

                    if (maxAbs > 1.0e-4F) {
                        norm = 1.0F / maxAbs;
                    }

                    if (norm > 0.0F) {
                        const float waveformAmplitude =
                            10.0F * connectionLevel;
                        const float waveformThickness = 1.2F;
                        const int periodSamples =
                            estimateWaveformPeriod(tempWave,
                                                   kWaveformPoints);

                        const bool tiled = !involvesSampleplay;
                        drawWaveformOnLine(
                            p1, p2, waveformAmplitude,
                            waveformThickness, tempWave,
                            periodSamples, norm, 64,
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
                    const float waveformAmplitude =
                        10.0F * connectionLevel;
                    const float waveformThickness = 1.2F;

                    float tempWave[kWaveformPoints]{};
                    float norm = 0.0F;

                    audioEngine_.getConnectionWaveformSnapshot(
                        connKey, tempWave, kWaveformPoints, 0.05);

                    float maxAbs = 0.0F;
                    for (int i = 0; i < kWaveformPoints; ++i) {
                        maxAbs = std::max(maxAbs, std::abs(tempWave[i]));
                    }

                    if (maxAbs > 1.0e-4F) {
                        norm = 1.0F / maxAbs;
                    }

                    if (norm > 0.0F) {
                        const int periodSamples =
                            estimateWaveformPeriod(tempWave,
                                                   kWaveformPoints);

                        // Use the same activeColour (including
                        // yellow when marked for cut) so that
                        // all audio-carrying connections share
                        // the same mute/hover visual behaviour.
                        g.setColour(activeColour);
                        const bool tiled = !involvesSampleplay;
                        drawWaveformOnLine(
                            p1, p2, waveformAmplitude,
                            waveformThickness, tempWave,
                            periodSamples, norm, 72,
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
    const float nodeRadius = 26.0F;

    auto getBodyColourForObject = [&modules](const rectai::ObjectInstance& obj,
                                             bool isMuted) {
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

        const auto bodyColour = getBodyColourForObject(object, isBodyMuted);

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

        // Node body.
        g.setColour(bodyColour);
        g.fillEllipse(cx - nodeRadius, cy - nodeRadius, nodeRadius * 2.0F,
                      nodeRadius * 2.0F);

        // Explicit outline so the circular base is always visible,
        // even when the fill colour is close to the background.
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

        // Side controls: left = curved bar (semi-arc) for Freq,
        // right = curved bar (semi-arc) for Gain, both hugging the
        // left/right side of the node.
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
                const double minBpm = 40.0;
                const double maxBpm = 400.0;
                const double clampedBpm =
                    juce::jlimit(minBpm, maxBpm, bpm_);
                const double norm =
                    (clampedBpm - minBpm) / (maxBpm - minBpm);
                freqValue = static_cast<float>(norm);
            } else if (moduleForObject->is<rectai::LoopModule>()) {
                // For Loop modules, the left bar represents the
                // currently selected sample slot via the normalised
                // "sample" parameter.
                freqValue = moduleForObject->GetParameterOrDefault(
                    "sample", 0.0F);
            } else {
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
                isTempoModule ||
                moduleForObject->is<rectai::LoopModule>();
            showGainControl = moduleForObject->uses_gain_control();
        }
        freqValue = juce::jlimit(0.0F, 1.0F, freqValue);
        gainValue = juce::jlimit(0.0F, 1.0F, gainValue);

        // Vertical span of the side arcs: make them almost
        // complete semi-circles while leaving a small gap at the
        // top so the radial audio line (and its waveform) can pass
        // cleanly between both bars. The same margin is used at the
        // bottom to keep the overall shape balanced.
        const float sliderMargin = 3.0F;
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

            const juce::Colour freqBackgroundColour =
                juce::Colours::white.withAlpha(0.35F);
            const juce::Colour freqForegroundColour =
                juce::Colours::white.withAlpha(1.0F);

            if (moduleForObject != nullptr &&
                moduleForObject->is<rectai::LoopModule>()) {
                // For Loop modules, render the left bar as four
                // discrete segments with a small transparent gap and
                // a triangle marker indicating the active slot.
                const int segmentsCount = 4;
                const float gap = 1.0F;

                // Active segment index derived from the normalised
                // sample parameter. We keep the raw parameter value
                // for the triangle so it can move smoothly with the
                // fiducial rotation, while the highlighted segment is
                // still quantised to four slots.
                float sampleParam = juce::jlimit(0.0F, 1.0F, freqValue);
                int activeIndex = static_cast<int>(sampleParam * 4.0F);
                if (activeIndex < 0) {
                    activeIndex = 0;
                } else if (activeIndex > 3) {
                    activeIndex = 3;
                }

                // Precompute the angle range covered by the visible
                // arc so that each Loop segment spans the same angle
                // and therefore has the same visual length.
                const float dyTop = sliderTop - cy;
                const float dyBottom = sliderBottom - cy;
                const float sinTop = juce::jlimit(-1.0F, 1.0F,
                                                  dyTop / ringRadius);
                const float sinBottom = juce::jlimit(-1.0F, 1.0F,
                                                     dyBottom / ringRadius);
                const float angleTop = std::asin(sinTop);
                const float angleBottom = std::asin(sinBottom);

                for (int i = 0; i < segmentsCount; ++i) {
                    // Draw segments from bottom (i = 0) to top
                    // (i = segmentsCount - 1) so that index 0
                    // corresponde a la parte baja de la barra y el
                    // último índice a la parte alta, alineados con el
                    // movimiento del triángulo.
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

                    // Apply a small angular shrink at both ends to
                    // create a 1px-ish gap between segments.
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

                    // Background bar segment.
                    g.setColour(freqBackgroundColour);
                    g.strokePath(segPath, juce::PathStrokeType(5.0F));

                    if (i == activeIndex) {
                        // Highlight the active segment with the
                        // foreground colour on top of the background.
                        g.setColour(freqForegroundColour);
                        g.strokePath(segPath, juce::PathStrokeType(5.0F));
                    }
                }

                // Triangle selector: triangle to the left of the bar,
                // sliding vertically along the full Loop control. We
                // keep the vertical position strictly proportional to
                // the continuous `sample` parameter so that there is
                // no visual jump when crossing segment boundaries.
                const float triY = juce::jmap(
                    sampleParam, 0.0F, 1.0F, sliderBottom, sliderTop);

                const float dyTri = triY - cy;
                const float insideTri =
                    ringRadius * ringRadius - dyTri * dyTri;
                if (insideTri > 0.0F) {
                    const float dxTri = std::sqrt(insideTri);
                    const float barX = cx - dxTri;

                    // Radial direction from the module centre to the
                    // active segment centre.
                    const float ux = (barX - cx) / ringRadius;
                    const float uy = (triY - cy) / ringRadius;

                    // Tip of the triangle sits exactly on the
                    // active segment, and the whole body extends
                    // *outside* the bar along the radial direction so
                    // it "wraps" the segment from the exterior.
                    const float triHeight = 7.0F;
                    const float halfWidth = 4.5F;

                    const juce::Point<float> tip(barX, triY);
                    const juce::Point<float> baseCenter(
                        tip.x + ux * triHeight,
                        tip.y + uy * triHeight);

                    // Tangent to the arc at the active segment,
                    // perpendicular to the radial.
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
                const bool freqIsFull = (freqValue >= 0.999F);
                const bool freqIsEmpty = (freqValue <= 0.001F);

                // When the control is at 0%, draw only the background
                // arc so the bar appears completely empty, avoiding any
                // tiny filled segment caused by clipping and
                // anti-aliasing.
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

                    // When the control is at 100%, render the entire arc
                    // with the foreground colour so that any small gap
                    // between the background and filled regions becomes
                    // invisible.
                    if (freqIsFull) {
                        effectiveFreqBackground = freqForegroundColour;
                    }

                    // Draw the full bar path as a darker background.
                    g.setColour(effectiveFreqBackground);
                    g.strokePath(freqArc,
                                 juce::PathStrokeType(5.0F));

                    // Draw the filled portion by clipping the same arc from
                    // the bottom of the bar up to the current value.
                    juce::Graphics::ScopedSaveState clipGuard(g);
                    const float clipPaddingX = 6.0F;
                    juce::Rectangle<int> filledClip(
                        static_cast<int>(cx - ringRadius - clipPaddingX),
                        static_cast<int>(handleY),
                        static_cast<int>(ringRadius * 2.0F +
                                         clipPaddingX * 2.0F),
                        static_cast<int>(sliderBottom - handleY + 4.0F));
                    g.reduceClipRegion(filledClip);

                    g.setColour(effectiveFreqForeground);
                    g.strokePath(freqArc,
                                 juce::PathStrokeType(5.0F));
                }
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
            g.strokePath(gainArc, juce::PathStrokeType(1.0F));

            // White handle: user-controlled module gain.
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

            // Grey handle: Sequencer-driven gain factor for
            // Oscillator modules. It follows the MIDI-controlled
            // volume but cannot rise above the white handle.
            if (sequencerControlsVolume_ && moduleForObject != nullptr &&
                moduleForObject->is<rectai::OscillatorModule>()) {
                float seqGain = 1.0F;
                const auto it = oscillatorSequencerGain_.find(
                    moduleForObject->id());
                if (it != oscillatorSequencerGain_.end()) {
                    seqGain = juce::jlimit(0.0F, 1.0F, it->second);
                }

                // Only show the grey handle when the Sequencer is
                // actually constraining the level (i.e. attempting
                // to lower volume). The effective Sequencer value is
                // clamped to the white handle so it never appears
                // above it.
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
                        g.setColour(juce::Colours::lightgrey.withAlpha(0.9F));
                        g.fillEllipse(greyX - 4.0F, greyY - 4.0F,
                                      8.0F, 8.0F);
                    }
                }
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

                // Dynamically tint the icon based on the background brightness.
                // If the body is dark, use white; if bright, use a dark
                // colour.
                const float brightness =
                    bodyColour.getBrightness();
                const juce::Colour iconTint =
                    brightness > 0.6F
                        ? juce::Colour::fromRGB(0x20, 0x20, 0x20)
                        : juce::Colours::white;
                g.setColour(iconTint.withAlpha(0.9F));

                const int destX = juce::roundToInt(iconBounds.getX());
                const int destY = juce::roundToInt(iconBounds.getY());
                const int destW = juce::roundToInt(iconBounds.getWidth());
                const int destH = juce::roundToInt(iconBounds.getHeight());

                // For Tempo modules, rotate the metronome image with the
                // tangible's own rotation so that visual feedback matches
                // the physical gesture used to change BPM.
                const bool isTempoIcon =
                    (moduleForObject->is<rectai::TempoModule>());
                if (isTempoIcon) {
                    juce::Graphics::ScopedSaveState tempoIconState(g);
                    // Invert the visual rotation so that the
                    // metronome image follows the physical gesture
                    // direction perceived on the table.
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

            // Dynamically tint the procedural icon/text based on brightness.
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

        // Sampleplay instrument title: when a Sampleplay module has
        // one or more instruments declared, render the active
        // instrument name to the right of the node body so that the
        // currently selected sound is visible on the table. The text
        // follows the same rotation as the module when inside the
        // musical area, so "right" is interpreted in the module's
        // local space.
        if (const auto* sampleModule =
            dynamic_cast<const rectai::SampleplayModule*>(
                moduleForObject)) {
            const auto* activeInstrument =
                sampleModule->active_instrument();
            if (activeInstrument != nullptr &&
                !activeInstrument->name.empty()) {
                const juce::String instrumentName(
                    activeInstrument->name);
                const float labelMargin = 10.0F;
                const float labelHeight = 18.0F;
                const float labelWidth = 140.0F;
                juce::Rectangle<float> labelBounds(
                    cx + nodeRadius + labelMargin,
                    cy - labelHeight * 0.5F,
                    labelWidth,
                    labelHeight);

                // Sampleplay instrument label fade: keep the text
                // fully visible for a short period after the module
                // is placed in the musical area or its instrument is
                // changed, then fade it out smoothly.
                double labelAlpha = 0.0;
                const auto itTime =
                    sampleplayLabelLastChangeSeconds_.find(
                        sampleModule->id());
                if (itTime !=
                    sampleplayLabelLastChangeSeconds_.end()) {
                    const double elapsed =
                        nowSeconds - itTime->second;
                    if (elapsed >= 0.0) {
                        if (elapsed <= 3.0) {
                            labelAlpha = 1.0;
                        } else if (elapsed <= 3.5) {
                            labelAlpha =
                                1.0 - (elapsed - 3.0) / 0.5;
                        }
                    }
                } else {
                    // First-time render: start the timer so that
                    // the label appears and will fade after a few
                    // seconds even for modules that were already on
                    // the table when the app started.
                    labelAlpha = 1.0;
                    const double firstNow = nowSeconds;
                    sampleplayLabelLastChangeSeconds_.emplace(
                        sampleModule->id(), firstNow);
                }

                if (labelAlpha > 0.0) {
                    const float brightness =
                        bodyColour.getBrightness();
                    const juce::Colour textColour =
                        brightness > 0.6F
                            ? juce::Colours::white
                            : juce::Colours::white;
                    const float alpha =
                        static_cast<float>(labelAlpha);
                    g.setColour(textColour.withAlpha(0.9F * alpha));
                    g.setFont(13.0F);
                    g.drawText(instrumentName, labelBounds,
                               juce::Justification::centredLeft,
                               false);
                }
            }
        }

        // Loop sample title: when a Loop module has one or more
        // loops declared, render the currently selected filename
        // (basename only) to the right of the node body. The label
        // appears for a few seconds after the selected slot changes
        // and then fades out.
        if (const auto* loopModule =
                dynamic_cast<const rectai::LoopModule*>(
                    moduleForObject)) {
            const auto& loops = loopModule->loops();
            if (!loops.empty()) {
                float sampleParam = loopModule->GetParameterOrDefault(
                    "sample", 0.0F);
                sampleParam = juce::jlimit(0.0F, 1.0F, sampleParam);
                int selectedIndex = static_cast<int>(sampleParam * 4.0F);
                if (selectedIndex < 0) {
                    selectedIndex = 0;
                } else if (selectedIndex > 3) {
                    selectedIndex = 3;
                }

                const rectai::LoopDefinition* chosen = nullptr;
                // Loops are already ordered by the loader; clamp to
                // available entries.
                if (selectedIndex < static_cast<int>(loops.size())) {
                    chosen = &loops[static_cast<std::size_t>(
                        selectedIndex)];
                } else {
                    chosen = &loops.front();
                }

                if (chosen != nullptr && !chosen->filename.empty()) {
                    juce::String name(chosen->filename);
                    // Strip any directory components to keep the
                    // label compact.
                    const int lastSlash = name.lastIndexOfAnyOf("/\\");
                    if (lastSlash >= 0 && lastSlash < name.length() - 1) {
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
                        // First-time render: bootstrap the timer so
                        // the label appears and fades on existing
                        // modules.
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
        }

#if !defined(NDEBUG)
        // Debug overlay: show the object/module id to the right of the
        // node so that we can quickly match dock entries and table
        // objects with their underlying ids when inspecting colours or
        // routing issues.
        {
            const juce::String debugId(object.logical_id());
            const float debugMargin = 6.0F;
            const float debugWidth = 60.0F;
            const float debugHeight = 16.0F;
            juce::Rectangle<float> debugBounds(
                cx + nodeRadius + debugMargin,
                cy - debugHeight * 0.5F,
                debugWidth,
                debugHeight);

            g.setColour(juce::Colours::white.withAlpha(0.8F));
            g.setFont(11.0F);
            g.drawText(debugId, debugBounds,
                       juce::Justification::centredLeft, false);
        }
#endif

        // Tempo controller overlay: show the current global BPM as a
        // small integer label positioned just outside the node circle,
        // so it remains legible. The underlying value is kept as a
        // double in `bpm_` but rendered without decimals, and only
        // appears briefly after tempo changes.
        if (moduleForObject != nullptr && bpmLabelAlpha > 0.0) {
            const auto* tempoModule =
                dynamic_cast<const rectai::TempoModule*>(
                    moduleForObject);
            if (tempoModule != nullptr) {
                const int bpmInt = static_cast<int>(
                    std::round(bpm_));
                juce::String bpmText(bpmInt);

                const float margin = 4.0F;
                juce::Rectangle<float> bpmBounds(
                    cx - nodeRadius,
                    cy - nodeRadius - 18.0F - margin,
                    nodeRadius * 2.0F, 18.0F);

                const float alpha = static_cast<float>(bpmLabelAlpha);
                g.setColour(
                    juce::Colours::white.withAlpha(0.9F * alpha));
                g.setFont(13.0F);
                g.drawText(bpmText, bpmBounds,
                           juce::Justification::topLeft, false);
            }
        }
    }

    // ---------------------------------------------------------------------
    // Sequencer widget overlays.
    // ---------------------------------------------------------------------
    for (const auto& entry : objects) {
        const auto& object = entry.second;
        if (object.docked() || object.logical_id() == rectai::MASTER_OUTPUT_ID) {
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
        const float dockWidth = calculateDockWidth(dockBounds.getWidth());
        juce::Rectangle<float> dockArea =
            dockBounds.removeFromRight(dockWidth);

        // Background panel.
        g.setColour(juce::Colour::fromRGB(0x40, 0x40, 0x40));  // #404040
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
                    getBodyColourForObject(*obj, isBodyMuted);

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
                        const auto& src = spriteIt->second.bounds;

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

                        // Draw the atlas icon within the computed bounds.
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
                // of colours and mapping respecto a los ids del .rtp.
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

void MainComponent::resized()
{
}
