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

            // Remember the first instrument name defined in the
            // .rtp file (if any) so we can use it as a preferred
            // default when rebuilding the instrument list from the
            // SoundFont itself.
            std::string preferredInstrumentName;
            const auto& rtpInstruments = sampleModule->instruments();
            if (!rtpInstruments.empty()) {
                preferredInstrumentName = rtpInstruments.front().name;
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

            // If the .rtp file declared a preferred instrument name
            // and it exists in the SoundFont presets, select it as
            // the active instrument. Otherwise default to the first
            // preset in the list.
            int defaultIndex = 0;
            if (!preferredInstrumentName.empty()) {
                for (std::size_t i = 0; i < instruments.size(); ++i) {
                    if (instruments[i].name == preferredInstrumentName) {
                        defaultIndex = static_cast<int>(i);
                        break;
                    }
                }
            }

            sampleModule->set_active_instrument_index(defaultIndex);
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
