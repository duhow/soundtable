// MainComponent core construction; rendering, input and audio logic live in
// the split translation units:
//  - MainComponent_Paint.cpp
//  - MainComponent_Input.cpp
//  - MainComponent_Audio.cpp
//  - MainComponent_Atlas.cpp

#include "MainComponent.h"

#include <array>
#include <cmath>
#include <memory>

#include "AudioEngine.h"
#include "core/AudioModules.h"
#include "core/ReactableRtpLoader.h"
#include "core/ReactableRtzLoader.h"
#include "core/SoundfontUtils.h"
#include "MainComponentHelpers.h"
#include "MainComponent_TextScroll.h"
#include "MainComponent_LoopFileBrowser.h"

void MainComponent::invalidateTableBackground()
{
    tableBackgroundDirty_ = true;
}

void MainComponent::renderTableBackgroundIfNeeded(
    const juce::Rectangle<int>& bounds)
{
    const int width = bounds.getWidth();
    const int height = bounds.getHeight();

    if (width <= 0 || height <= 0) {
        return;
    }

    const bool sizeChanged = tableBackgroundCache_.isNull() ||
                             tableBackgroundCache_.getWidth() != width ||
                             tableBackgroundCache_.getHeight() != height;

    if (!sizeChanged && !tableBackgroundDirty_) {
        return;
    }

    // The table background is fully opaque (solid black + coloured
    // disc and gradient ring), so we can use an RGB image without
    // alpha to reduce per-pixel blending work when blitting.
    tableBackgroundCache_ = juce::Image(juce::Image::RGB, width, height,
                                        false);
    juce::Graphics g(tableBackgroundCache_);

    // Match the previous behaviour: solid black background behind
    // the coloured table disc and its soft outer ring.
    g.fillAll(juce::Colours::black);

    const auto floatBounds = bounds.toFloat();
    const auto centre = floatBounds.getCentre();
    const float tableRadius =
        0.45F * std::min(floatBounds.getWidth(), floatBounds.getHeight());

    const bool hasAudioInitError = audioEngine_.hasInitialisationError();

    const juce::Colour tableColour = hasAudioInitError
                                         ? juce::Colour::fromRGB(
                                               0x80, 0x1a, 0x1a)
                                         : juce::Colour::fromRGB(
                                               0x00, 0x1a, 0x80);

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
        borderRing.setUsingNonZeroWinding(false);

        juce::ColourGradient borderGradient(tableColour, centre.x, centre.y,
                                            juce::Colours::black, centre.x,
                                            centre.y + outerRadius, true);

        const double innerStop =
            static_cast<double>(tableRadius / outerRadius);
        borderGradient.addColour(innerStop, tableColour);
        borderGradient.addColour(1.0, juce::Colours::black);

        g.setGradientFill(borderGradient);
        g.fillPath(borderRing);
    }

    g.setColour(tableColour);
    g.fillEllipse(centre.x - tableRadius, centre.y - tableRadius,
                  tableRadius * 2.0F, tableRadius * 2.0F);

    tableBackgroundDirty_ = false;
}

void MainComponent::repaintWithRateLimit()
{
    const double nowSeconds =
        juce::Time::getMillisecondCounterHiRes() / 1000.0;
    constexpr double kMaxRepaintsPerSecond = 60.0;
    const double minRepaintInterval = 1.0 / kMaxRepaintsPerSecond;

    if (nowSeconds - lastRepaintSeconds_ >= minRepaintInterval) {
        repaint();
        lastRepaintSeconds_ = nowSeconds;
    }
}

void MainComponent::invalidateDockBackground()
{
    dockBackgroundDirty_ = true;
}

void MainComponent::renderDockBackgroundIfNeeded(
    const juce::Rectangle<int>& dockBounds)
{
    const int width = dockBounds.getWidth();
    const int height = dockBounds.getHeight();

    if (width <= 0 || height <= 0) {
        return;
    }

    const bool sizeChanged = dockBackgroundCache_.isNull() ||
                             dockBackgroundCache_.getWidth() != width ||
                             dockBackgroundCache_.getHeight() != height;

    if (!sizeChanged && !dockBackgroundDirty_) {
        return;
    }

    dockBackgroundCache_ =
        juce::Image(juce::Image::RGB, width, height, false);
    juce::Graphics g(dockBackgroundCache_);

    g.fillAll(juce::Colours::transparentBlack);

    juce::Rectangle<float> area(0.0F, 0.0F,
                                static_cast<float>(width),
                                static_cast<float>(height));

    g.setColour(juce::Colour::fromRGB(0x40, 0x40, 0x40));
    g.fillRoundedRectangle(area, 10.0F);
    g.setColour(juce::Colours::white.withAlpha(0.25F));
    g.drawRoundedRectangle(area, 10.0F, 1.5F);

    g.setColour(juce::Colours::white.withAlpha(0.7F));
    g.setFont(15.0F);
    const float titleHeight = 24.0F;
    juce::Rectangle<float> titleArea =
        area.removeFromTop(titleHeight);
    g.drawText("Dock", titleArea.reduced(6.0F, 0.0F),
               juce::Justification::centredLeft, false);

    dockBackgroundDirty_ = false;
}

juce::Rectangle<float> MainComponent::getModulePanelBounds(
    const rectai::ObjectInstance& object,
    const juce::Rectangle<float>& bounds) const
{
    const auto centrePos = objectTableToScreen(object, bounds);
    const float cx = centrePos.x;
    const float cy = centrePos.y;

    // Base geometry for the panel in the same (pre-rotated)
    // coordinate system que el cuerpo del módulo y sus barras
    // laterales. El pintado del nodo aplica después una
    // AffineTransform de rotación alrededor de (cx, cy), de modo que
    // este rectángulo también rota y se mantiene siempre en el lado
    // "derecho" local del módulo.
    constexpr float kNodeRadius = 26.0F;
    const float ringRadius = kNodeRadius + 10.0F;

    // Coloca el panel muy cerca de la barra de volumen (Gain),
    // apenas separado del anillo de controles laterales para que se
    // perciba visualmente unido al módulo.
    const float horizontalOffset = ringRadius + 10.0F;
    const float verticalLift = 10.0F;

    const float px = cx + horizontalOffset;
    const float py = cy - (kModulePanelHeight * 0.5F) - verticalLift;

    return {px, py, kModulePanelWidth, kModulePanelHeight};
}

MainComponent::MainComponent(AudioEngine& audioEngine,
                             juce::String initialSessionPath)
    : audioEngine_(audioEngine)
{
    setSize(1280, 720);
    // MainComponent fully covers its bounds with an opaque background
    // (black backdrop + coloured table image), so hint JUCE that it
    // does not require clearing or blending with underlying content.
    setOpaque(true);
    invalidateTableBackground();

    // Resolve the root of the com.reactable content tree so that we
    // have a stable base directory for Sessions/ and Samples/ when
    // importing .rtz archives.
    juce::File comReactableRoot;
    {
        const juce::File defaultRtp =
            rectai::ui::loadFile("Resources/default.rtp");
        if (defaultRtp.existsAsFile()) {
            comReactableRoot = defaultRtp.getParentDirectory()
                                      .getParentDirectory();
        } else {
            const juce::File cwd =
                juce::File::getCurrentWorkingDirectory();
            comReactableRoot = cwd.getChildFile("com.reactable");
        }
    }

    // Base directory for Samples/ content used by the Loop file
    // browser. It is resolved even if assets are missing so that the
    // browser can still operate against an empty directory.
    samplesRootDir_ = comReactableRoot.getChildFile("Samples");

    // Load initial Reactable patch. If an explicit session file was
    // provided on the command line, try that first; otherwise fall
    // back to com.reactable/Resources/default.rtp. If loading fails
    // (e.g., when running tests without assets), fall back to the
    // small hardcoded demo scene used originally.
    const auto loadPatchFromFile =
        [this, comReactableRoot](const juce::File& patchFile) {
        using rectai::LoadReactablePatchFromFile;
        using rectai::LoadReactableSessionFromRtz;
        using rectai::ReactablePatchMetadata;

        if (!patchFile.existsAsFile()) {
            return false;
        }

        rectai::ReactablePatchMetadata metadata;
        std::string error;

        scene_ = rectai::Scene{};

        const juce::String extension =
            patchFile.getFileExtension().toLowerCase();

        bool ok = false;
        if (extension == ".rtz") {
            ok = LoadReactableSessionFromRtz(
                patchFile.getFullPathName().toStdString(),
                comReactableRoot.getFullPathName().toStdString(),
                scene_, &metadata, &error);
        } else {
            ok = LoadReactablePatchFromFile(
                patchFile.getFullPathName().toStdString(), scene_,
                &metadata, &error);
        }
        if (ok) {
            masterColour_ =
                rectai::ui::colourFromArgb(metadata.master_colour_argb);
            masterMuted_ = metadata.master_muted;

            // Log basic project metadata (authors and title) when a
            // session is successfully loaded.
            juce::String author =
                juce::String(metadata.author_name).trim();
            juce::String title =
                juce::String(metadata.patch_name).trim();

            juce::String info;
            if (author.isNotEmpty() && title.isNotEmpty()) {
                info = author + " - " + title;
            } else if (author.isNotEmpty()) {
                info = author;
            } else if (title.isNotEmpty()) {
                info = title;
            }

            juce::String logMessage("Loaded project: ");
            if (info.isNotEmpty()) {
                logMessage += info;
            }
            juce::Logger::writeToLog(logMessage);

            // Seed per-connection mute state from the Scene model.
            // Reactable hardlinks can declare muted="1" and the
            // loader maps that into Connection::muted. Mirror that
            // into mutedConnections_ so that the visual and audio
            // mute state matches the session on load.
            mutedConnections_.clear();
            for (const auto& conn : scene_.connections()) {
                if (!conn.muted) {
                    continue;
                }
                const std::string key =
                    rectai::ui::makeConnectionKey(conn);
                mutedConnections_.insert(key);
            }

            // If the loaded scene contains a Tempo module, initialise
            // the global BPM from its `tempo` parameter so that visual
            // pulses and transport widgets match the patch settings.
            const auto& modules = scene_.modules();
            for (const auto& [id, modulePtr] : modules) {
                if (modulePtr == nullptr) {
                    continue;
                }

                const auto* tempoModule =
                    dynamic_cast<const rectai::TempoModule*>(
                        modulePtr.get());
                if (tempoModule == nullptr) {
                    continue;
                }

                const float tempoValue = tempoModule->GetParameterOrDefault(
                    "tempo", 120.0F);
                bpm_ = rectai::TempoModule::ClampBpm(tempoValue);
                break;
            }
            return true;
        }

        juce::Logger::writeToLog(
            juce::String("[rectai-core] Failed to load session: ") +
            patchFile.getFullPathName() + " (" + juce::String(error) +
            ")");
        return false;
    };

    const auto loadSampleplaySoundfonts = [this]() {
        const auto& modules = scene_.modules();

        // Track whether we have already configured a SoundFont path
        // in the audio engine so that multiple Sampleplay modules
        // sharing el mismo archivo no causen recargas redundantes.
        bool audioEngineSoundfontSet = false;

        for (const auto& [id, modulePtr] : modules) {
            juce::ignoreUnused(id);
            if (modulePtr == nullptr) {
                continue;
            }

            auto* sampleModule =
                dynamic_cast<rectai::SampleplayModule*>(
                    modulePtr.get());
            if (sampleModule == nullptr) {
                continue;
            }

            // Remember the instrument names declared in the .rtp
            // (index 0 → drums, index 1 → synths, etc.) that we will
            // later map to actual SoundFont presets by name.
            std::vector<std::string> preferredInstrumentNames;
            const auto& rtpInstruments = sampleModule->instruments();
            preferredInstrumentNames.reserve(rtpInstruments.size());
            for (const auto& inst : rtpInstruments) {
                preferredInstrumentNames.push_back(inst.name);
            }

            std::string rawName = sampleModule->raw_soundfont_name();
            if (rawName.empty()) {
                // Fallback to the default Reactable soundfont name
                // used by the bundled patches.
                rawName = "default.sf2";
            }

            const juce::String relativePath =
                juce::String("Soundfonts/") + rawName;
            const juce::File sf2File =
                rectai::ui::loadFile(relativePath);
            if (!sf2File.existsAsFile()) {
                juce::Logger::writeToLog(
                    juce::String("[rectai-core] Sampleplay: soundfont not "
                                 "found: ") +
                    sf2File.getFullPathName());
                continue;
            }

            std::string error;
            const bool ok = sampleModule->LoadSoundfont(
                sf2File.getFullPathName().toStdString(), &error);
            if (!ok) {
                juce::Logger::writeToLog(
                    juce::String("[rectai-core] Sampleplay: failed to load "
                                 "soundfont: ") +
                    sf2File.getFullPathName() + " (" + error + ")");
                continue;
            }

            // Inform the AudioEngine about the SoundFont path so it
            // can initialise its internal FluidSynth-backed synth.
            if (!audioEngineSoundfontSet) {
                audioEngine_.setSampleplaySoundfont(
                    sampleModule->soundfont_path());
                audioEngineSoundfontSet = true;
            }

            // Once the SoundFont is loaded and validated, enumerate
            // all presets using FluidSynth and rebuild the
            // SampleplayModule instrument list from the actual
            // contents of the .sf2 file, ignoring the list in the
            // .rtp (except for the preferred default name).
            std::vector<rectai::SoundfontPreset> presets;
            std::string enumError;
            if (!rectai::EnumerateSoundfontPresets(
                    sampleModule->soundfont_path(), presets,
                    &enumError)) {
                juce::Logger::writeToLog(
                    juce::String("[rectai-core] Sampleplay: failed to "
                                 "enumerate presets: ") +
                    sf2File.getFullPathName() + " (" + enumError + ")");
                continue;
            }

            auto& instruments = sampleModule->mutable_instruments();
            instruments.clear();
            instruments.reserve(presets.size());

            for (const auto& p : presets) {
                rectai::SampleInstrument inst;
                inst.name = p.name;
                inst.bank = p.bank;
                inst.program = p.program;
                instruments.push_back(std::move(inst));
            }

            // Build default preset indices per logical bank: for each
            // <instrument> declared in the .rtp, try to find a matching
            // preset name in the SoundFont and remember its index.
            std::vector<int> defaultPresetIndices(
                preferredInstrumentNames.size(), -1);

            auto containsIgnoreCase =
                [](const std::string& haystack,
                   const std::string& needle) noexcept {
                    if (needle.empty()) {
                        return true;
                    }
                    auto it = std::search(
                        haystack.begin(), haystack.end(),
                        needle.begin(), needle.end(),
                        [](unsigned char ch1, unsigned char ch2) {
                            return static_cast<int>(
                                       std::tolower(ch1)) ==
                                   static_cast<int>(
                                       std::tolower(ch2));
                        });
                    return it != haystack.end();
                };

            // Pre-compute indices of presets that live on banks other
            // than 0. In many GM/GS SoundFonts, drum kits are located
            // on banks >= 128.
            std::vector<int> nonZeroBankPresetIndices;
            std::vector<int> highBankPresetIndices;
            nonZeroBankPresetIndices.reserve(instruments.size());
            highBankPresetIndices.reserve(instruments.size());
            for (std::size_t i = 0; i < instruments.size(); ++i) {
                const int bank = instruments[i].bank;
                if (bank != 0) {
                    nonZeroBankPresetIndices.push_back(
                        static_cast<int>(i));
                    if (bank >= 128) {
                        highBankPresetIndices.push_back(
                            static_cast<int>(i));
                    }
                }
            }

            for (std::size_t bankIdx = 0;
                 bankIdx < preferredInstrumentNames.size(); ++bankIdx) {
                const auto& name = preferredInstrumentNames[bankIdx];
                if (name.empty()) {
                    continue;
                }

                const bool isDrumLogicalBank =
                    containsIgnoreCase(name, "drumset") ||
                    containsIgnoreCase(name, "drum");

                int matchedIndex = -1;

                // 1) Exact match against preset name.
                for (std::size_t i = 0; i < instruments.size(); ++i) {
                    if (instruments[i].name == name) {
                        matchedIndex = static_cast<int>(i);
                        break;
                    }
                }

                // 2) For drum banks without an exact match, first try to
                //    assign a preset from a physical bank different from 0,
                //    prioritising higher banks (>= 128) typically used for
                //    percussion in GM/GS SoundFonts.
                if (matchedIndex < 0 && isDrumLogicalBank) {
                    if (!highBankPresetIndices.empty()) {
                        matchedIndex = highBankPresetIndices.front();
                    } else if (!nonZeroBankPresetIndices.empty()) {
                        matchedIndex =
                            nonZeroBankPresetIndices.front();
                    }
                }

                // 3) If we still have no mapping for a drum bank and there
                //    are no alternative physical banks, make a final generic
                //    attempt by searching for presets whose name suggests
                //    percussion. This keeps compatibility with SF2 files
                //    that only use bank 0 but include kits such as
                //    "Warehouse Percussion", without depending on specific
                //    names.
                if (matchedIndex < 0 && isDrumLogicalBank) {
                    for (std::size_t i = 0; i < instruments.size(); ++i) {
                        const auto& presetName = instruments[i].name;
                        if (containsIgnoreCase(presetName,
                                               "percussion") ||
                            containsIgnoreCase(presetName,
                                               "kit") ||
                            containsIgnoreCase(presetName,
                                               "drum")) {
                            matchedIndex = static_cast<int>(i);
                            break;
                        }
                    }
                }

                // 4) If we still do not have a mapping, leave the index at
                //    -1. Later logic will pick the first known valid preset
                //    or, ultimately, index 0.
                defaultPresetIndices[bankIdx] = matchedIndex;
            }

            sampleModule->set_default_preset_indices(
                std::move(defaultPresetIndices));

            // Choose the active instrument according to the logical
            // channel declared in the .rtp (attribute "channel"). If
            // there is no valid preset for that channel, fall back to
            // the first valid preset we know about, or index 0.
            const auto& defaults =
                sampleModule->default_preset_indices();
            int chosenIndex = -1;
            const int channel = sampleModule->channel();
            if (channel >= 0 &&
                channel < static_cast<int>(defaults.size())) {
                const int idx =
                    defaults[static_cast<std::size_t>(channel)];
                if (idx >= 0 &&
                    idx < static_cast<int>(instruments.size())) {
                    chosenIndex = idx;
                }
            }

            if (chosenIndex < 0) {
                for (const int idx : defaults) {
                    if (idx >= 0 &&
                        idx < static_cast<int>(instruments.size())) {
                        chosenIndex = idx;
                        break;
                    }
                }
            }

            if (chosenIndex < 0 && !instruments.empty()) {
                chosenIndex = 0;
            }

            if (chosenIndex >= 0) {
                sampleModule->set_active_instrument_index(chosenIndex);
            }
        }
    };

    // Bridge TUIO cursor events from the OSC receiver into the same
    // interaction model used by regular mouse input. Las coordenadas
    // llegan normalizadas en [0,1] sobre todo el componente.
    trackingOscReceiver_.setTuioCursorCallbacks(
        [this](float normX, float normY, std::int32_t /*sessionId*/) {
            handleTuioCursorDown(normX, normY);
        },
        [this](float normX, float normY, std::int32_t /*sessionId*/) {
            handleTuioCursorMove(normX, normY);
        },
        [this](std::int32_t /*sessionId*/) {
            handleTuioCursorUp();
        });

    // Track high-level input activity (OSC vs TUIO) so that the UI
    // can expose a small traffic indicator near the dock. The
    // callback runs on the JUCE message thread, so it can safely
    // touch UI state and trigger repaints.
    trackingOscReceiver_.setActivityCallback(
        [this](TrackingOscReceiver::ActivityKind kind) {
            const double nowSeconds =
                juce::Time::getMillisecondCounterHiRes() / 1000.0;

            if (kind == TrackingOscReceiver::ActivityKind::kTuio) {
                lastInputActivityKind_ = InputActivityKind::kTuio;
            } else {
                lastInputActivityKind_ = InputActivityKind::kOsc;
            }

            lastInputActivitySeconds_ = nowSeconds;
            inputActivityPulseSeconds_ = nowSeconds;

            repaintWithRateLimit();
        });

    // Lazy-initialised helper for the Loop file browser. It is
    // constructed once we know the Samples/ root directory.
    loopFileBrowser_ = std::make_unique<LoopFileBrowser>(
        loopFileBrowsers_, loopFileLists_, samplesRootDir_,
        [this]() { repaintWithRateLimit(); },
        [this](const std::string& moduleId) {
            markLoopSampleLabelActive(moduleId);
        },
        [this](const std::string& moduleId) -> rectai::LoopModule* {
            auto* base = scene_.FindModule(moduleId);
            return dynamic_cast<rectai::LoopModule*>(base);
        },
        [this](const std::string& moduleId,
               int slotIndex,
               const std::string& fullPath,
               float beats,
               std::string* error) {
            return audioEngine_.loadLoopSampleFromFile(
                moduleId, slotIndex, fullPath, beats, error);
        },
        [this](const std::string& moduleId, int slotIndex) {
            return audioEngine_.getLoopSampleBeats(moduleId,
                                                   slotIndex);
        });

    const auto loadLoopSamples = [this]() {
        const auto& modules = scene_.modules();

        for (const auto& [id, modulePtr] : modules) {
            juce::ignoreUnused(id);
            if (modulePtr == nullptr) {
                continue;
            }

            auto* loopModule =
                dynamic_cast<rectai::LoopModule*>(modulePtr.get());
            if (loopModule == nullptr) {
                continue;
            }

            // Order loops by the `order` field and load up to the
            // first four entries. Filenames are resolved relative to
            // com.reactable/Samples/.
            std::vector<rectai::LoopDefinition> loops =
                loopModule->loops();
            std::sort(loops.begin(), loops.end(),
                      [](const rectai::LoopDefinition& a,
                         const rectai::LoopDefinition& b) {
                          return a.order < b.order;
                      });

            int slotIndex = 0;
            for (const auto& loopDef : loops) {
                if (slotIndex >= 4) {
                    break;
                }
                if (loopDef.filename.empty()) {
                    continue;
                }

                const juce::String relativePath =
                    juce::String("Samples/") +
                    juce::String(loopDef.filename);
                const juce::File audioFile =
                    rectai::ui::loadFile(relativePath);
                if (!audioFile.existsAsFile()) {
                    juce::Logger::writeToLog(
                        juce::String("[rectai-core] Loop: sample not "
                                     "found: ") +
                        audioFile.getFullPathName());
                    ++slotIndex;
                    continue;
                }

                std::string error;
                const bool ok = audioEngine_.loadLoopSampleFromFile(
                    loopModule->id(), slotIndex,
                    audioFile.getFullPathName().toStdString(),
                    loopDef.beats, &error);
                if (!ok) {
                    juce::Logger::writeToLog(
                        juce::String("[rectai-core] Loop: failed to load "
                                     "sample: ") +
                        audioFile.getFullPathName() + " (" +
                        juce::String(error) + ")");
                }

                ++slotIndex;
            }
        }
    };

    bool loaded = false;

    // 1) If an explicit session path was provided on startup, try
    //    to load it and validate that it is a proper Reactable
    //    .rtp project.
    if (initialSessionPath.isNotEmpty()) {
        const juce::File userFile =
            juce::File::getCurrentWorkingDirectory().getChildFile(
                initialSessionPath);
        if (!userFile.existsAsFile()) {
            juce::Logger::writeToLog(
                juce::String("[rectai-core] Session file not found: ") +
                userFile.getFullPathName());
        } else {
            loaded = loadPatchFromFile(userFile);
        }
    }

    // 2) If no file was provided or loading the explicit file
    //    failed, fall back to the bundled default patch.
    if (!loaded) {
        const juce::File defaultFile =
            rectai::ui::loadFile("Resources/default.rtp");
        if (defaultFile.existsAsFile()) {
            loaded = loadPatchFromFile(defaultFile);
        }
    }

    // 3) If we still have no valid scene (e.g. running without
    //    assets), use the original minimal demo scene.
    if (!loaded) {
        // Fallback: example scene with a couple of modules and an
        // explicit Output/master so that connection-level mute via
        // radials and AudioGraph-based routing behave consistently
        // with real Reactable patches.
        auto osc1 = std::make_unique<rectai::OscillatorModule>("osc1");
        auto filter1 =
            std::make_unique<rectai::FilterModule>("filter1");
        auto output =
            std::make_unique<rectai::OutputModule>(rectai::MASTER_OUTPUT_ID);

        (void)scene_.AddModule(std::move(osc1));
        (void)scene_.AddModule(std::move(filter1));
        (void)scene_.AddModule(std::move(output));

        // Basic chain Osc -> Filter -> Output(master).
        {
            rectai::Connection c1{
                .from_module_id = "osc1",
                .from_port_name = "out",
                .to_module_id = "filter1",
                .to_port_name = "in",
                .is_hardlink = false};
            (void)scene_.AddConnection(c1);

            rectai::Connection c2{
                .from_module_id = "filter1",
                .from_port_name = "out",
                .to_module_id = rectai::MASTER_OUTPUT_ID,
                .to_port_name = "in",
                .is_hardlink = false};
            (void)scene_.AddConnection(c2);

            // Auto-wire the Oscillator directly to Output so that
            // its radial to the master exists in the Scene model and
            // can be muted via line-cutting gestures, mirroring the
            // behaviour of patches loaded from .rtp.
            rectai::Connection c3{
                .from_module_id = "osc1",
                .from_port_name = "out",
                .to_module_id = rectai::MASTER_OUTPUT_ID,
                .to_port_name = "in",
                .is_hardlink = false};
            (void)scene_.AddConnection(c3);
        }

        // Normalized positions on the table. The Output tangible is
        // represented as an object with logical_id MASTER_OUTPUT_ID but remains
        // invisible in the UI (MainComponent_Paint filters it out).
        scene_.UpsertObject(
            rectai::ObjectInstance(1, "osc1", 0.3F, 0.5F, 0.0F));
        scene_.UpsertObject(rectai::ObjectInstance(
            2, "filter1", 0.7F, 0.5F, 0.0F));
        scene_.UpsertObject(rectai::ObjectInstance(
            -1, rectai::MASTER_OUTPUT_ID, 0.5F, 0.1F, 0.0F));
    }

    // After the scene has been populated, recompute the cached flag
    // that marks which objects lie inside the musical area so that
    // early paint/audio passes can rely on this attribute without
    // recomputing geometry.
    refreshInsideMusicAreaFlags();

    // Attempt to resolve and load SoundFont2 files for any
    // Sampleplay modules present in the patch. The loader stores the
    // raw filename from the .rtp file; here we look it up under
    // com.reactable/Soundfonts/ and validate it via
    // SampleplayModule::LoadSoundfont.
    loadSampleplaySoundfonts();

    // Resolve and load audio files for any Loop modules present in
    // the patch. The .rtp loader has already populated the
    // LoopDefinition list; here we resolve filenames under
    // com.reactable/Samples/ and hand them to the AudioEngine.
    loadLoopSamples();

    // Reset the global loop beat counter so that Loop modules start
    // from a well-defined transport origin for this session.
    audioEngine_.resetLoopBeatCounter(0U);

    // Load Reactable icon atlas (atlas_2048.png + atlas_2048.xml) so that
    // modules can be rendered with their original icons instead of only
    // text labels or procedural shapes. If loading fails, the UI will
    // gracefully fall back to the existing vector-based icons and labels.
    (void)loadAtlasResources();

    // Periodically map scene state to audio parameters and refresh the
    // UI. Use a relatively high update rate so that waveform
    // visualisations and other animations feel responsive and the
    // Sequencer step triggering jitter remains low relative to the
    // audio transport.
    const double nowSeconds =
        juce::Time::getMillisecondCounterHiRes() / 1000.0;
    lastTimerSeconds_ = nowSeconds;
    lastRepaintSeconds_ = nowSeconds;
    // Increasing the timer frequency reduces the temporal
    // quantisation of Sequencer steps (which are currently
    // advanced from `timerCallback` using `transportBeats()`) and
    // makes each note fire closer to its ideal beat. 240 Hz
    // corresponds to a nominal resolution of ~4.2 ms.
    startTimerHz(240);
}

juce::Image MainComponent::getCachedAtlasIcon(const std::string& iconId,
                                              int destWidth,
                                              int destHeight)
{
    if (!atlasLoaded_ || iconId.empty() || !atlasImage_.isValid() ||
        destWidth <= 0 || destHeight <= 0) {
        return {};
    }

    const auto spriteIt = atlasSprites_.find(iconId);
    if (spriteIt == atlasSprites_.end()) {
        return {};
    }

    const auto key = iconId + "#" + std::to_string(destWidth) + "x" +
                     std::to_string(destHeight);

    const auto cacheIt = atlasIconCache_.find(key);
    if (cacheIt != atlasIconCache_.end() && cacheIt->second.isValid() &&
        cacheIt->second.getWidth() == destWidth &&
        cacheIt->second.getHeight() == destHeight) {
        return cacheIt->second;
    }

    const auto& src = spriteIt->second.bounds;
    if (src.getWidth() <= 0 || src.getHeight() <= 0) {
        return {};
    }

    juce::Image scaled(juce::Image::SingleChannel, destWidth, destHeight,
                       true);
    {
        juce::Graphics g(scaled);
        // Copy the alpha mask from the atlas into the target image,
        // resampling to the requested size. The resulting
        // SingleChannel image can then be tinted at paint time.
        g.drawImage(atlasImage_, 0, 0, destWidth, destHeight, src.getX(),
                    src.getY(), src.getWidth(), src.getHeight());
    }

    atlasIconCache_[key] = scaled;
    return scaled;
}

void MainComponent::resized()
{
    invalidateTableBackground();
    invalidateDockBackground();
    juce::Component::resized();
}

void MainComponent::markSampleplayInstrumentLabelActive(
    const std::string& moduleId)
{
    const double nowSeconds =
        juce::Time::getMillisecondCounterHiRes() / 1000.0;
    sampleplayLabelLastChangeSeconds_[moduleId] = nowSeconds;
}

void MainComponent::markLoopSampleLabelActive(
    const std::string& moduleId)
{
    const double nowSeconds =
        juce::Time::getMillisecondCounterHiRes() / 1000.0;
    loopLabelLastChangeSeconds_[moduleId] = nowSeconds;
}

void MainComponent::markDelayNoteSegmentActive(
    const std::string& moduleId, int segmentIndex)
{
    if (segmentIndex < 0) {
        return;
    }

    const double nowSeconds =
        juce::Time::getMillisecondCounterHiRes() / 1000.0;

    auto& state = delayNoteOverlays_[moduleId];
    state.segmentIndex = segmentIndex;
    state.lastChangeSeconds = nowSeconds;
}

void MainComponent::ensureLoopFileBrowserInitialised(
    const std::string& moduleId, rectai::LoopModule* loopModule)
{
    if (loopFileBrowser_ == nullptr) {
        return;
    }
    loopFileBrowser_->ensureInitialised(moduleId, loopModule);
}

void MainComponent::rebuildLoopFileBrowserEntries(
    const std::string& moduleId, rectai::LoopModule* loopModule)
{
    if (loopFileBrowser_ == nullptr) {
        return;
    }
    loopFileBrowser_->rebuildEntries(moduleId, loopModule);
}

void MainComponent::onLoopFileSelectionChanged(
    const std::string& moduleId, const int rowIndex)
{
    if (loopFileBrowser_ == nullptr) {
        return;
    }
    loopFileBrowser_->handleSelectionChanged(moduleId, rowIndex);
}

rectai::ui::TextScrollList*
MainComponent::getOrCreateLoopFileList(const std::string& moduleId)
{
    if (loopFileBrowser_ == nullptr) {
        return nullptr;
    }
    return loopFileBrowser_->getOrCreateList(moduleId);
}

rectai::ui::TextScrollList*
MainComponent::getOrCreateTempoPresetList(const std::string& moduleId)
{
    auto it = tempoPresetLists_.find(moduleId);
    if (it != tempoPresetLists_.end()) {
        return it->second.get();
    }

    auto list = std::make_unique<rectai::ui::TextScrollList>();
    list->setMaxVisibleItems(6);
    list->setRowHeight(18.0F);

    // Selection callback: update the global BPM and the Tempo module
    // parameter whenever a preset is chosen from the TextScrollList.
    list->setOnSelectionChanged(
        [this, moduleId](int index) {
            const auto& presets = rectai::TempoModule::bpm_presets();
            if (index < 0 ||
                index >= static_cast<int>(presets.size())) {
                return;
            }

            const float newBpm = rectai::TempoModule::ClampBpm(
                presets[static_cast<std::size_t>(index)].bpm);

            if (newBpm == bpm_) {
                return;
            }

            bpm_ = newBpm;
            bpmLastChangeSeconds_ =
                juce::Time::getMillisecondCounterHiRes() / 1000.0;

            scene_.SetModuleParameter(moduleId, "tempo", bpm_);

            repaintWithRateLimit();
        });

    rectai::ui::TextScrollList* raw = list.get();
    tempoPresetLists_.emplace(moduleId, std::move(list));
    return raw;
}

rectai::ui::XYControl*
MainComponent::getOrCreateXYControl(const std::string& moduleId)
{
    auto it = xyControls_.find(moduleId);
    if (it != xyControls_.end()) {
        return it->second.get();
    }

    auto control = std::make_unique<rectai::ui::XYControl>();
    rectai::ui::XYControl* raw = control.get();
    xyControls_.emplace(moduleId, std::move(control));
    return raw;
}
