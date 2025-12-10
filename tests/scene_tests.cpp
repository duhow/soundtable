#include <cassert>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>

#include "core/Scene.h"
#include "core/AudioModules.h"
#include "core/SceneSerialization.h"
#include "core/ReactableRtpLoader.h"

// Very small tests for the Scene/Module/Connection model.
// They run as a normal binary and are integrated with CTest.

using rectai::Connection;
using rectai::FilterModule;
using rectai::ObjectInstance;
using rectai::OscillatorModule;
using rectai::ReactablePatchMetadata;
using rectai::Scene;
using rectai::SampleplayModule;

int main()
{
    Scene scene;

    auto osc = std::make_unique<OscillatorModule>("osc1");
    auto filter = std::make_unique<FilterModule>("filter1");

    // Modules can be added, but duplicates should not be inserted.
    assert(scene.AddModule(std::move(osc)));
    auto duplicatedOsc = std::make_unique<OscillatorModule>("osc1");
    assert(!scene.AddModule(std::move(duplicatedOsc)));

    assert(scene.AddModule(std::move(filter)));

    // Valid connection between existing modules.
    Connection connection{.from_module_id = "osc1",
                          .from_port_name = "out",
                          .to_module_id = "filter1",
                          .to_port_name = "in"};
    assert(scene.AddConnection(connection));

    // Duplicated connection should not be added.
    assert(!scene.AddConnection(connection));

    // Remove connection.
    assert(scene.RemoveConnection("osc1", "out", "filter1", "in"));
    assert(!scene.RemoveConnection("osc1", "out", "filter1", "in"));

    // Manage tangible objects.
    scene.UpsertObject(ObjectInstance(1, "osc1", 0.5F, 0.5F, 0.0F));
    assert(scene.objects().size() == 1U);

    scene.UpsertObject(ObjectInstance(1, "osc1", 0.7F, 0.5F, 0.0F));
    // Upsert should replace the existing object instead of inserting a new one.
    assert(scene.objects().size() == 1U);

    scene.RemoveObject(1);
    assert(scene.objects().empty());

    // Oscillator waveform cycling should update icon id.
    {
        OscillatorModule oscWave("oscWave");
        assert(oscWave.icon_id() == std::string("oscillator_sine"));

        oscWave.cycle_waveform();
        assert(oscWave.icon_id() == std::string("oscillator_saw"));

        oscWave.cycle_waveform();
        assert(oscWave.icon_id() == std::string("oscillator_square"));

        oscWave.cycle_waveform();
        assert(oscWave.icon_id() == std::string("oscillator_noise"));

        oscWave.cycle_waveform();
        assert(oscWave.icon_id() == std::string("oscillator_sine"));
    }

    // Basic serialization smoke test.
    const auto serialized = SerializeScene(scene);
    assert(!serialized.empty());
    assert(serialized.find("rectai_scene_v1") != std::string::npos);

    // Reactable .rtp loader smoke test (string-based).
    {
        // Minimal Reactable patch with a single Oscillator tangible.
        const char* kRtp =
            "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n"
            "<reactablepatch>\n"
            "  <background color=\"0,0,0\" texture=\"\" alpha=\"1\" rotation=\"0\" revolution=\"0\" random=\"0\" />\n"
            "  <tangibles>\n"
            "    <tangible type=\"Oscillator\" id=\"46\" x=\"0.1\" y=\"0.2\" angle=\"90\" color=\"ffffffff\" docked=\"0\" muted=\"0\" point=\"0\" subtype=\"sine\" amp=\"0.5\" sweep=\"0\" midifreq=\"44\">\n"
            "      <envelope attack=\"0\" decay=\"0\" duration=\"25\" points_x=\"0,1\" points_y=\"0,1\" release=\"0\" />\n"
            "    </tangible>\n"
            "  </tangibles>\n"
            "  <author name=\"TestAuthor\" />\n"
            "  <patch name=\"TestPatch\" />\n"
            "</reactablepatch>\n";

        Scene loaded_scene;
        ReactablePatchMetadata metadata;
        std::string error;
        const bool ok = rectai::LoadReactablePatchFromString(kRtp, loaded_scene,
                                                             &metadata, &error);
        assert(ok);
        assert(error.empty());

        // One module and one object should have been created.
        assert(loaded_scene.modules().size() == 1U);
        assert(loaded_scene.objects().size() == 1U);

        // Module id and ObjectInstance logical_id must match the tangible id.
        const auto& modules = loaded_scene.modules();
        const auto module_it = modules.find("46");
        assert(module_it != modules.end());
        const rectai::AudioModule* module = module_it->second.get();
        assert(module != nullptr);
        assert(module->id() == "46");

        const auto* oscModule =
            dynamic_cast<const OscillatorModule*>(module);
        assert(oscModule != nullptr);
        assert(oscModule->icon_id() == std::string("oscillator_sine"));

        const auto& objects = loaded_scene.objects();
        const auto obj_it = objects.find(46);
        assert(obj_it != objects.end());
        const ObjectInstance& obj = obj_it->second;
        assert(obj.logical_id() == "46");
        assert(obj.x() == 0.1F);
        assert(obj.y() == 0.2F);

        // Angle is given in degrees in the .rtp and converted to radians.
        const float expected_angle_rad = 3.14159265F / 2.0F;  // 90 deg.
        assert(obj.angle_radians() > expected_angle_rad - 0.01F);
        assert(obj.angle_radians() < expected_angle_rad + 0.01F);

        // Metadata should be populated.
        assert(metadata.author_name == "TestAuthor");
        assert(metadata.patch_name == "TestPatch");
    }

    // Sampleplay soundfont loading.
    {
        // Create a minimal SF2-like file with the correct RIFF/sfbk header.
        const char* kSf2Path = "test_soundfont.sf2";
        {
            std::ofstream out(kSf2Path, std::ios::binary);
            assert(out.is_open());
            const char header[12] = {'R', 'I', 'F', 'F', 0, 0, 0, 0,
                                     's', 'f', 'b', 'k'};
            out.write(header, sizeof(header));
        }

        SampleplayModule sample("sp1");
        std::string error;
        const bool ok = sample.LoadSoundfont(kSf2Path, &error);
        assert(ok);
        assert(error.empty());
        assert(sample.has_soundfont());
        assert(sample.soundfont_path() == kSf2Path);

        // Clean up the temporary file; ignore errors.
        (void)std::remove(kSf2Path);
    }

    // Hardlink-based connection creation.
    {
        const char* kRtpWithHardlink =
            "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n"
            "<reactablepatch>\n"
            "  <background color=\"0,0,0\" texture=\"\" alpha=\"1\" rotation=\"0\" revolution=\"0\" random=\"0\" />\n"
            "  <tangibles>\n"
            "    <tangible type=\"Output\" id=\"-1\" x=\"0\" y=\"0\" angle=\"0\" color=\"ffffffff\" docked=\"0\" muted=\"0\" point=\"0\" />\n"
            "    <tangible type=\"Delay\" id=\"10\" x=\"0.1\" y=\"0.2\" angle=\"0\" color=\"000000ff\" docked=\"0\" muted=\"0\" point=\"0\" subtype=\"pingpong\" delay=\"0.66\" fb=\"0.5\" sweep=\"0\">\n"
            "      <envelope attack=\"0\" decay=\"0\" duration=\"25\" points_x=\"0,1\" points_y=\"0,1\" release=\"0\" />\n"
            "      <hardlink to=\"-1\" />\n"
            "    </tangible>\n"
            "  </tangibles>\n"
            "  <author name=\"TestAuthor\" />\n"
            "  <patch name=\"HardlinkPatch\" />\n"
            "</reactablepatch>\n";

        Scene loaded_scene;
        ReactablePatchMetadata metadata;
        std::string error;
        const bool ok = rectai::LoadReactablePatchFromString(
            kRtpWithHardlink, loaded_scene, &metadata, &error);
        assert(ok);
        assert(error.empty());

        // Expect two modules (Output and Delay) and two objects.
        assert(loaded_scene.modules().size() == 2U);
        assert(loaded_scene.objects().size() == 2U);

        // One connection should have been created from Delay (10) to Output (-1).
        const auto& connections = loaded_scene.connections();
        assert(connections.size() == 1U);
        const Connection& c = connections.front();
        assert(c.from_module_id == "10");
        assert(c.from_port_name == "out");
        assert(c.to_module_id == "-1");
        assert(c.to_port_name == "in");
        // Connections derived from <hardlink> should be marked as
        // hardlinks in the Scene model.
        assert(c.is_hardlink);
    }

    std::cout << "rectai-core-tests: OK" << std::endl;
    return 0;
}
