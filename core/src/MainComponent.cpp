// MainComponent core construction; rendering, input and audio logic live in
// the split translation units:
//  - MainComponent_Paint.cpp
//  - MainComponent_Input.cpp
//  - MainComponent_Audio.cpp
//  - MainComponent_Atlas.cpp

#include "MainComponent.h"

#include <memory>

#include "AudioEngine.h"
#include "core/AudioModules.h"
#include "core/ReactableRtpLoader.h"
#include "core/SoundfontUtils.h"
#include "MainComponentHelpers.h"

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

        const juce::File patchFile =
            rectai::ui::loadFile("Resources/default.rtp");
        if (!patchFile.existsAsFile()) {
            return false;
        }

        rectai::ReactablePatchMetadata metadata;
        std::string error;

        scene_ = rectai::Scene{};
        const bool ok = LoadReactablePatchFromFile(
            patchFile.getFullPathName().toStdString(), scene_, &metadata,
            &error);
        if (ok) {
            masterColour_ =
                rectai::ui::colourFromArgb(metadata.master_colour_argb);
            masterMuted_ = metadata.master_muted;

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
                const double clamped = juce::jlimit(40.0, 400.0,
                                                    static_cast<double>(
                                                        tempoValue));
                bpm_ = clamped;
                break;
            }
            return true;
        }

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
            // file in order; this sequence define logical banks
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
            // <instrument> declared en el .rtp, try to find a matching
            // preset name en el SoundFont y recordar su índice.
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
            // than 0. In muchos SoundFonts GM/GS, los kits de percusión
            // se sitúan en bancos >= 128.
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

                // 2) For drum banks sin match exacto, intentar primero
                //    asignar un preset de un banco físico distinto de 0,
                //    priorizando bancos altos (>= 128) típicamente usados
                //    para percusión en SoundFonts GM/GS.
                if (matchedIndex < 0 && isDrumLogicalBank) {
                    if (!highBankPresetIndices.empty()) {
                        matchedIndex = highBankPresetIndices.front();
                    } else if (!nonZeroBankPresetIndices.empty()) {
                        matchedIndex =
                            nonZeroBankPresetIndices.front();
                    }
                }

                // 3) Si seguimos sin mapping para un banco de drums y no
                //    hay bancos físicos alternativos, hacemos un último
                //    intento genérico buscando presets cuyo nombre sugiera
                //    percusión. Esto mantiene compatibilidad con SF2 que
                //    sólo usan bank 0 pero incluyen kits como "Warehouse
                //    Percussion", sin depender de nombres concretos.
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

                // 4) Si aún no tenemos mapping, dejamos el índice en -1.
                //    La lógica posterior elegirá el primer preset válido
                //    conocido o, en última instancia, el índice 0.
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

    // After the scene has been populated, attempt to resolve and load
    // SoundFont2 files for any Sampleplay modules present in the
    // patch. The loader stores the raw filename from the .rtp file;
    // here we look it up under com.reactable/Soundfonts/ and validate
    // it via SampleplayModule::LoadSoundfont.
    loadSampleplaySoundfonts();

    // Load Reactable icon atlas (atlas_2048.png + atlas_2048.xml) so that
    // modules can be rendered with their original icons instead of only
    // text labels or procedural shapes. If loading fails, the UI will
    // gracefully fall back to the existing vector-based icons and labels.
    (void)loadAtlasResources();

    // Periodically map scene state to audio parameters and refresh the
    // UI. Use a relatively high update rate so that waveform
    // visualisations and other animations feel responsive.
    lastTimerSeconds_ =
        juce::Time::getMillisecondCounterHiRes() / 1000.0;
    startTimerHz(120);
}
