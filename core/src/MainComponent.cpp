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
            return true;
        }

        return false;
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
