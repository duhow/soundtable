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

using rectai::AudioModule;
using rectai::Connection;
using rectai::FilterModule;
using rectai::ObjectInstance;
using rectai::OscillatorModule;
using rectai::OutputModule;
using rectai::ReactablePatchMetadata;
using rectai::Scene;
using rectai::SampleplayModule;
using rectai::SequencerModule;
using rectai::SequencerPreset;
using rectai::SequencerStep;
using rectai::TempoModule;
using rectai::TonalizerModule;
using rectai::VolumeModule;

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

        // The Oscillator should expose the `midifreq` parameter as
        // loaded from the .rtp patch.
        const float midiFromParam = oscModule->GetParameterOrDefault(
            "midifreq", -1.0F);
        assert(midiFromParam == 44.0F);

        // The loader must initialise the normalised `freq` parameter
        // from `midifreq` using the module's frequency mapping so
        // that the resulting Hz is close to the expected MIDI pitch.
        const float freqParam = oscModule->GetParameterOrDefault(
            "freq", -1.0F);
        assert(freqParam >= 0.0F && freqParam <= 1.0F);

        const double baseHz = oscModule->base_frequency_hz();
        const double rangeHz = oscModule->frequency_range_hz();
        const double effectiveHz =
            baseHz + rangeHz * static_cast<double>(freqParam);

        const double targetHz =
            440.0 * std::pow(2.0, (44.0 - 69.0) / 12.0);
        assert(effectiveHz > targetHz * 0.9);
        assert(effectiveHz < targetHz * 1.1);

        // Envelope from the <envelope> tag should be applied both to
        // the internal structure and as module parameters.
        const auto& env = oscModule->envelope();
        assert(env.attack == 0.0F);
        assert(env.decay == 0.0F);
        assert(env.duration == 25.0F);
        assert(env.release == 0.0F);

        assert(oscModule->GetParameterOrDefault("attack", -1.0F) ==
               env.attack);
        assert(oscModule->GetParameterOrDefault("decay", -1.0F) ==
               env.decay);
        assert(oscModule->GetParameterOrDefault("duration", -1.0F) ==
               env.duration);
        assert(oscModule->GetParameterOrDefault("release", -1.0F) ==
               env.release);

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

    // Sequencer v1: pulse-based loading from .rtp.
    {
        // Minimal Reactable patch with a single Sequencer tangible
        // in legacy (version 1) mode. The sequencer sends pulses
        // based on `steps` and `volumes`; melodic information from
        // `step_frequencies` should be ignored for version 1.
        const char* kRtpSequencerV1 =
            "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n"
            "<reactablepatch>\n"
            "  <tangibles>\n"
            "    <tangible type=\"Sequencer\" id=\"6\" x=\"0.0\" y=\"0.0\" angle=\"0\" color=\"000000ff\" docked=\"0\" muted=\"0\" point=\"0\" subtype=\"sequencer\" current_track=\"0\" autoseq_on=\"0\" noteedit_on=\"0\" duration=\"0.75\" num_tracks=\"6\" offset=\"0\">\n"
            "      <sequence rows=\"1,1,1,1,1,1,1,1,1,1,1,1,1\" speed=\"1\" speed_type=\"binary\" step_frequencies=\"0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\" steps=\"1,0,0,1,0,0,0,1,0,1,0,0,1,0,0,0\" tenori0=\"0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\" volumes=\"1,1,1,0.25,1,1,1,0.75,1,0.1,1,1,0.6,1,1,1\" />\n"
            "    </tangible>\n"
            "  </tangibles>\n"
            "</reactablepatch>\n";

        Scene loaded_scene;
        ReactablePatchMetadata metadata;
        std::string error;
        const bool ok = rectai::LoadReactablePatchFromString(
            kRtpSequencerV1, loaded_scene, &metadata, &error);
        assert(ok);
        assert(error.empty());

        // One Sequencer module and one object should have been created.
        assert(loaded_scene.modules().size() == 1U);
        assert(loaded_scene.objects().size() == 1U);

        const auto& modules_seq = loaded_scene.modules();
        const auto module_it_seq = modules_seq.find("6");
        assert(module_it_seq != modules_seq.end());
        const AudioModule* baseModule = module_it_seq->second.get();
        assert(baseModule != nullptr);

        const auto* seqModule = dynamic_cast<const SequencerModule*>(baseModule);
        assert(seqModule != nullptr);

        // Version attribute is absent in the tangible, so the
        // Sequencer must default to version 1 (pulse mode).
        assert(seqModule->version() == 1);

        // Tracks loaded from <sequence> must be reflected into the
        // fixed-size presets via SyncPresetsFromTracks(). In v1 the
        // enabled steps and their velocities come from `steps` and
        // `volumes`, and melodic information is ignored (constant
        // pitch for all pulses).
        const auto& tracks = seqModule->tracks();
        assert(!tracks.empty());

        const SequencerPreset& preset0 = seqModule->preset(0);
        // Indices 0, 3, 7, 9 and 12 are set to 1 in `steps`.
        const int expectedEnabledIndices[] = {0, 3, 7, 9, 12};
        for (int i = 0; i < SequencerPreset::kNumSteps; ++i) {
            const SequencerStep& step =
                preset0.steps[static_cast<std::size_t>(i)];

            bool shouldBeEnabled = false;
            for (int enabledIndex : expectedEnabledIndices) {
                if (i == enabledIndex) {
                    shouldBeEnabled = true;
                    break;
                }
            }

            if (shouldBeEnabled) {
                assert(step.enabled);
                // Volumes must be respected and clamped to [0,1].
                assert(step.velocity01 >= 0.0F && step.velocity01 <= 1.0F);
            } else {
                assert(!step.enabled);
                // In v1 disabled steps expose zero velocity.
                assert(step.velocity01 == 0.0F);
            }

            // In version 1 all steps share the same pitch by design
            // (no melody is encoded in step_frequencies), so the
            // sequencer behaves as a pulse generator.
            assert(step.pitch == 60);
        }
    }

    // Colours parsed from .rtp (RGB and RGB+alpha).
    {
        const char* kRtpColours =
            "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n"
            "<reactablepatch>\n"
            "  <tangibles>\n"
            // Loop modules with 6-digit RGB colours.
            "    <tangible id=\"24\" type=\"Loop\" color=\"d4cd06\" x=\"0\" y=\"0\" angle=\"0\" docked=\"1\" />\n"
            "    <tangible id=\"29\" type=\"Loop\" color=\"e76200\" x=\"0\" y=\"0\" angle=\"0\" docked=\"1\" />\n"
            "    <tangible id=\"34\" type=\"Loop\" color=\"2366e1\" x=\"0\" y=\"0\" angle=\"0\" docked=\"1\" />\n"
            "    <tangible id=\"39\" type=\"Loop\" color=\"23cb43\" x=\"0\" y=\"0\" angle=\"0\" docked=\"1\" />\n"
            // Delay with 8-digit RGB+alpha colour (black, fully opaque).
            "    <tangible id=\"10\" type=\"Delay\" color=\"000000ff\" x=\"0\" y=\"0\" angle=\"0\" docked=\"0\" />\n"
            "  </tangibles>\n"
            "</reactablepatch>\n";

        Scene loaded_scene;
        ReactablePatchMetadata metadata;
        std::string error;
        const bool ok = rectai::LoadReactablePatchFromString(
            kRtpColours, loaded_scene, &metadata, &error);
        assert(ok);
        assert(error.empty());

        const auto& modules = loaded_scene.modules();
        assert(modules.size() == 5U);

        auto expectColour = [&modules](const char* id,
                                       std::uint32_t expected) {
            const auto it = modules.find(id);
            assert(it != modules.end());
            const auto* m = it->second.get();
            assert(m != nullptr);
            assert(m->colour_argb() == expected);
        };

        // 6-digit RGB should become 0xFFRRGGBB.
        expectColour("24", 0xFFD4CD06U);
        expectColour("29", 0xFFE76200U);
        expectColour("34", 0xFF2366E1U);
        expectColour("39", 0xFF23CB43U);

        // 8-digit RRGGBBAA should be interpreted as RGB+alpha
        // and converted to ARGB internally. 000000ff => opaque black.
        expectColour("10", 0xFF000000U);
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
        assert(c.to_module_id == rectai::MASTER_OUTPUT_ID);
        assert(c.to_port_name == "in");
        // Connections derived from <hardlink> should be marked as
        // hardlinks in the Scene model.
        assert(c.is_hardlink);
        // When no muted attribute is present on <hardlink>, the
        // connection should start unmuted.
        assert(!c.muted);

        // Attempts to add additional connections with the same
        // endpoints (whether as hardlinks or dynamic connections)
        // must be rejected so that Delay(10) only ever has a single
        // connection to the master Output (-1).
        Connection duplicateHard{
            .from_module_id = "10",
            .from_port_name = "out",
            .to_module_id = rectai::MASTER_OUTPUT_ID,
            .to_port_name = "in",
            .is_hardlink = true};
        assert(!loaded_scene.AddConnection(duplicateHard));

        Connection duplicateDynamic{
            .from_module_id = "10",
            .from_port_name = "out",
            .to_module_id = rectai::MASTER_OUTPUT_ID,
            .to_port_name = "in",
            .is_hardlink = false};
        assert(!loaded_scene.AddConnection(duplicateDynamic));
    }

    // Hardlink-based connection creation with muted="1".
    {
        const char* kRtpWithMutedHardlink =
            "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n"
            "<reactablepatch>\n"
            "  <background color=\"0,0,0\" texture=\"\" alpha=\"1\" rotation=\"0\" revolution=\"0\" random=\"0\" />\n"
            "  <tangibles>\n"
            "    <tangible type=\"Output\" id=\"-1\" x=\"0\" y=\"0\" angle=\"0\" color=\"ffffffff\" docked=\"0\" muted=\"0\" point=\"0\" />\n"
            "    <tangible type=\"Delay\" id=\"10\" x=\"0.1\" y=\"0.2\" angle=\"0\" color=\"000000ff\" docked=\"0\" muted=\"0\" point=\"0\" subtype=\"pingpong\" delay=\"0.66\" fb=\"0.5\" sweep=\"0\">\n"
            "      <envelope attack=\"0\" decay=\"0\" duration=\"25\" points_x=\"0,1\" points_y=\"0,1\" release=\"0\" />\n"
            "      <hardlink to=\"-1\" muted=\"1\" />\n"
            "    </tangible>\n"
            "  </tangibles>\n"
            "  <author name=\"TestAuthor\" />\n"
            "  <patch name=\"HardlinkMutedPatch\" />\n"
            "</reactablepatch>\n";

        Scene loaded_scene;
        ReactablePatchMetadata metadata;
        std::string error;
        const bool ok = rectai::LoadReactablePatchFromString(
            kRtpWithMutedHardlink, loaded_scene, &metadata, &error);
        assert(ok);
        assert(error.empty());

        const auto& connections = loaded_scene.connections();
        assert(connections.size() == 1U);
        const Connection& c = connections.front();
        assert(c.is_hardlink);
        assert(c.muted);
    }

    // Auto-wired connections from audio-capable modules to Output/master.
    {
        const char* kRtpWithMasterAuto =
            "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n"
            "<reactablepatch>\n"
            "  <background color=\"0,0,0\" texture=\"\" alpha=\"1\" rotation=\"0\" revolution=\"0\" random=\"0\" />\n"
            "  <tangibles>\n"
            "    <tangible type=\"Output\" id=\"-1\" x=\"0\" y=\"0\" angle=\"0\" color=\"ffffffff\" docked=\"0\" muted=\"0\" point=\"0\" />\n"
            "    <tangible type=\"Oscillator\" id=\"46\" x=\"0.2\" y=\"0.3\" angle=\"0\" color=\"ffffffff\" docked=\"0\" muted=\"0\" point=\"0\" subtype=\"sine\" amp=\"0.5\" sweep=\"0\" midifreq=\"44\">\n"
            "      <envelope attack=\"0\" decay=\"0\" duration=\"25\" points_x=\"0,1\" points_y=\"0,1\" release=\"0\" />\n"
            "    </tangible>\n"
            "  </tangibles>\n"
            "  <author name=\"TestAuthor\" />\n"
            "  <patch name=\"MasterAutoPatch\" />\n"
            "</reactablepatch>\n";

        Scene loaded_scene;
        ReactablePatchMetadata metadata;
        std::string error;
        const bool ok = rectai::LoadReactablePatchFromString(
            kRtpWithMasterAuto, loaded_scene, &metadata, &error);
        assert(ok);
        assert(error.empty());

        // Expect two modules (Output and Oscillator) and two objects.
        assert(loaded_scene.modules().size() == 2U);
        assert(loaded_scene.objects().size() == 2U);

        // One non-hardlink connection should have been auto-created
        // from the Oscillator (46) to Output (-1).
        const auto& connections2 = loaded_scene.connections();
        assert(connections2.size() == 1U);
        const Connection& c2 = connections2.front();
        assert(c2.from_module_id == "46");
        assert(c2.from_port_name == "out");
        assert(c2.to_module_id == rectai::MASTER_OUTPUT_ID);
        assert(c2.to_port_name == "in");
        assert(!c2.is_hardlink);
    }

    // Global controller modules (Volume, Tempo, Tonalizer) must not
    // participate in the explicit connection graph (no dynamic links
    // or hardlinks in Scene::connections).
    {
        Scene s;

        auto vol = std::make_unique<VolumeModule>("vol1");
        auto tempo = std::make_unique<TempoModule>("tempo1");
        auto tonalizer = std::make_unique<TonalizerModule>("ton1");
        auto osc = std::make_unique<OscillatorModule>("osc1");

        assert(s.AddModule(std::move(vol)));
        assert(s.AddModule(std::move(tempo)));
        assert(s.AddModule(std::move(tonalizer)));
        assert(s.AddModule(std::move(osc)));

        // Any connection involving a global controller should be rejected.
        Connection c1{.from_module_id = "vol1",
                      .from_port_name = "out",
                      .to_module_id = "osc1",
                      .to_port_name = "in"};
        Connection c2{.from_module_id = "osc1",
                      .from_port_name = "out",
                      .to_module_id = "vol1",
                      .to_port_name = "in"};
        Connection c3{.from_module_id = "tempo1",
                      .from_port_name = "out",
                      .to_module_id = "osc1",
                      .to_port_name = "in"};
        Connection c4{.from_module_id = "osc1",
                      .from_port_name = "out",
                      .to_module_id = "ton1",
                      .to_port_name = "in"};

        assert(!s.AddConnection(c1));
        assert(!s.AddConnection(c2));
        assert(!s.AddConnection(c3));
        assert(!s.AddConnection(c4));
        assert(s.connections().empty());
    }

    // Non-hardlink connections per module: at most one outgoing
    // dynamic connection is allowed.
    {
        Scene s;

        auto osc = std::make_unique<OscillatorModule>("osc1");
        auto filter1 = std::make_unique<FilterModule>("filter1");
        auto filter2 = std::make_unique<FilterModule>("filter2");

        assert(s.AddModule(std::move(osc)));
        assert(s.AddModule(std::move(filter1)));
        assert(s.AddModule(std::move(filter2)));

        // First dynamic connection from osc1 to filter1 should be
        // accepted.
        Connection d1{.from_module_id = "osc1",
                      .from_port_name = "out",
                      .to_module_id = "filter1",
                      .to_port_name = "in",
                      .is_hardlink = false};
        assert(s.AddConnection(d1));

        // Second dynamic connection from osc1 to filter2 should be
        // rejected due to the per-module limit.
        Connection d2{.from_module_id = "osc1",
                      .from_port_name = "out",
                      .to_module_id = "filter2",
                      .to_port_name = "in",
                      .is_hardlink = false};
        assert(!s.AddConnection(d2));

        const auto& conns = s.connections();
        std::size_t dynamicCount = 0;
        std::size_t hardlinkCount = 0;
        for (const auto& c : conns) {
            if (c.from_module_id != "osc1") {
                continue;
            }
            if (c.is_hardlink) {
                ++hardlinkCount;
            } else {
                ++dynamicCount;
            }
        }

        assert(dynamicCount == 1U);
        assert(hardlinkCount == 0U);
    }

    // Sampleplay modules must follow the same dynamic connection
    // limit rules as Oscillator: at most one non-hardlink outgoing
    // connection to other modules, while still allowing an
    // additional non-hardlink connection to the master Output (-1).
    {
        Scene s;

        auto sample = std::make_unique<SampleplayModule>("sp1");
        auto filter1 = std::make_unique<FilterModule>("filter1");
        auto filter2 = std::make_unique<FilterModule>("filter2");
        auto output = std::make_unique<OutputModule>(rectai::MASTER_OUTPUT_ID);

        assert(s.AddModule(std::move(sample)));
        assert(s.AddModule(std::move(filter1)));
        assert(s.AddModule(std::move(filter2)));
        assert(s.AddModule(std::move(output)));

        // First dynamic connection from sp1 to filter1 should be
        // accepted.
        Connection d1{.from_module_id = "sp1",
                      .from_port_name = "out",
                      .to_module_id = "filter1",
                      .to_port_name = "in",
                      .is_hardlink = false};
        assert(s.AddConnection(d1));

        // Second dynamic connection from sp1 to filter2 should be
        // rejected due to the per-module limit.
        Connection d2{.from_module_id = "sp1",
                      .from_port_name = "out",
                      .to_module_id = "filter2",
                      .to_port_name = "in",
                      .is_hardlink = false};
        assert(!s.AddConnection(d2));

        // An additional non-hardlink connection to Output (-1)
        // must still be accepted.
        Connection toMaster{.from_module_id = "sp1",
                            .from_port_name = "out",
                            .to_module_id = rectai::MASTER_OUTPUT_ID,
                            .to_port_name = "in",
                            .is_hardlink = false};
        assert(s.AddConnection(toMaster));

        const auto& conns = s.connections();
        std::size_t dynamicCount = 0;
        std::size_t hardlinkCount = 0;
        for (const auto& c : conns) {
            if (c.from_module_id != "sp1") {
                continue;
            }
            if (c.is_hardlink) {
                ++hardlinkCount;
            } else {
                ++dynamicCount;
            }
        }

        // Expect exactly one dynamic connection to another module
        // plus one dynamic connection to the master Output (-1),
        // and no hardlinks.
        assert(dynamicCount == 2U);
        assert(hardlinkCount == 0U);
    }

    // Hardlink connections from the same source module are not
    // counted towards the dynamic limit and multiple distinct
    // hardlinks are permitted.
    {
        Scene s;

        auto osc = std::make_unique<OscillatorModule>("osc1");
        auto filter1 = std::make_unique<FilterModule>("filter1");
        auto filter2 = std::make_unique<FilterModule>("filter2");

        assert(s.AddModule(std::move(osc)));
        assert(s.AddModule(std::move(filter1)));
        assert(s.AddModule(std::move(filter2)));

        Connection h1{.from_module_id = "osc1",
                      .from_port_name = "out",
                      .to_module_id = "filter1",
                      .to_port_name = "in",
                      .is_hardlink = true};
        Connection h2{.from_module_id = "osc1",
                      .from_port_name = "out",
                      .to_module_id = "filter2",
                      .to_port_name = "in",
                      .is_hardlink = true};

        assert(s.AddConnection(h1));
        assert(s.AddConnection(h2));

        const auto& conns = s.connections();
        std::size_t dynamicCount = 0;
        std::size_t hardlinkCount = 0;
        for (const auto& c : conns) {
            if (c.from_module_id != "osc1") {
                continue;
            }
            if (c.is_hardlink) {
                ++hardlinkCount;
            } else {
                ++dynamicCount;
            }
        }

        assert(dynamicCount == 0U);
        assert(hardlinkCount == 2U);
    }

    // Dynamic connections can be promoted to hardlinks and demoted
    // back to dynamic connections without violating per-module
    // routing limits.
    {
        Scene s;

        auto osc = std::make_unique<OscillatorModule>("osc1");
        auto filter = std::make_unique<FilterModule>("filter1");

        assert(s.AddModule(std::move(osc)));
        assert(s.AddModule(std::move(filter)));

        // Start with a single dynamic connection osc1 -> filter1.
        Connection dyn{.from_module_id = "osc1",
                       .from_port_name = "out",
                       .to_module_id = "filter1",
                       .to_port_name = "in",
                       .is_hardlink = false};
        assert(s.AddConnection(dyn));

        // Promote that connection to a hardlink by removing the
        // dynamic edge and inserting a new hardlink edge with the
        // same endpoints. Scene::AddConnection must accept this
        // because hardlinks are not counted towards the dynamic
        // per-module limit.
        assert(s.RemoveConnection(dyn));

        Connection hard{.from_module_id = "osc1",
                        .from_port_name = "out",
                        .to_module_id = "filter1",
                        .to_port_name = "in",
                        .is_hardlink = true};
        assert(s.AddConnection(hard));

        assert(s.connections().size() == 1U);
        const Connection& cHard = s.connections().front();
        assert(cHard.is_hardlink);

        // Demote back to a dynamic connection by removing the
        // hardlink and re-inserting a non-hardlink edge. This
        // mirrors the behaviour used when toggling collisions in
        // the UI: a promoted hardlink returns to its original
        // dynamic state.
        assert(s.RemoveConnection(hard));

        Connection dyn2{.from_module_id = "osc1",
                        .from_port_name = "out",
                        .to_module_id = "filter1",
                        .to_port_name = "in",
                        .is_hardlink = false};
        assert(s.AddConnection(dyn2));

        assert(s.connections().size() == 1U);
        const Connection& cDyn = s.connections().front();
        assert(!cDyn.is_hardlink);
    }

    // A module can have one dynamic connection to another module and
    // an additional non-hardlink connection to the master Output (-1).
    {
        Scene s;

        auto osc = std::make_unique<OscillatorModule>("osc1");
        auto filter = std::make_unique<FilterModule>("filter1");
        auto output = std::make_unique<OutputModule>(rectai::MASTER_OUTPUT_ID);

        assert(s.AddModule(std::move(osc)));
        assert(s.AddModule(std::move(filter)));
        assert(s.AddModule(std::move(output)));

        // Dynamic connection osc1 -> filter1.
        Connection dyn{.from_module_id = "osc1",
                       .from_port_name = "out",
                       .to_module_id = "filter1",
                       .to_port_name = "in",
                       .is_hardlink = false};
        assert(s.AddConnection(dyn));

        // Additional non-hardlink connection osc1 -> Output (-1)
        // should be accepted and must not count against the
        // per-module dynamic limit.
        Connection toMaster{.from_module_id = "osc1",
                            .from_port_name = "out",
                            .to_module_id = rectai::MASTER_OUTPUT_ID,
                            .to_port_name = "in",
                            .is_hardlink = false};
        assert(s.AddConnection(toMaster));

        const auto& conns = s.connections();
        std::size_t oscOutCount = 0;
        for (const auto& c : conns) {
            if (c.from_module_id == "osc1") {
                ++oscOutCount;
            }
        }

        // Expect both connections to coexist.
        assert(oscOutCount == 2U);
    }

    // When a module already has a hardlink, it must not be allowed
    // to add a new dynamic connection; only further hardlinks are
    // permitted.
    {
        Scene s;

        auto osc = std::make_unique<OscillatorModule>("osc1");
        auto filter1 = std::make_unique<FilterModule>("filter1");
        auto filter2 = std::make_unique<FilterModule>("filter2");

        assert(s.AddModule(std::move(osc)));
        assert(s.AddModule(std::move(filter1)));
        assert(s.AddModule(std::move(filter2)));

        Connection h1{.from_module_id = "osc1",
                      .from_port_name = "out",
                      .to_module_id = "filter1",
                      .to_port_name = "in",
                      .is_hardlink = true};
        assert(s.AddConnection(h1));

        // After the first hardlink, a dynamic connection from the
        // same source must be rejected.
        Connection d1{.from_module_id = "osc1",
                      .from_port_name = "out",
                      .to_module_id = "filter2",
                      .to_port_name = "in",
                      .is_hardlink = false};
        assert(!s.AddConnection(d1));

        // Additional hardlinks from the same source remain valid.
        Connection h2{.from_module_id = "osc1",
                      .from_port_name = "out",
                      .to_module_id = "filter2",
                      .to_port_name = "in",
                      .is_hardlink = true};
        assert(s.AddConnection(h2));

        const auto& conns = s.connections();
        std::size_t dynamicCount = 0;
        std::size_t hardlinkCount = 0;
        for (const auto& c : conns) {
            if (c.from_module_id != "osc1") {
                continue;
            }
            if (c.is_hardlink) {
                ++hardlinkCount;
            } else {
                ++dynamicCount;
            }
        }

        assert(dynamicCount == 0U);
        assert(hardlinkCount == 2U);
    }

    // Volume module defaults: global volume starts at ~90%.
    {
        VolumeModule volDefault("vol_default");
        const float volumeParam =
            volDefault.GetParameterOrDefault("volume", 0.0F);
        assert(volumeParam > 0.89F && volumeParam < 0.91F);
    }

    // Volume module loaded from .rtp with volume="90" debe normalizarse a 0.9.
    {
        const char* kRtpVolume =
            "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n"
            "<reactablepatch>\n"
            "  <background color=\"0,0,0\" texture=\"\" alpha=\"1\" rotation=\"0\" revolution=\"0\" random=\"0\" />\n"
            "  <tangibles>\n"
            "    <tangible id=\"1\" type=\"Volume\" subtype=\"volume\" angle=\"0\" volume=\"90\" x=\"1.1\" y=\"0.4\" docked=\"1\" dock_pos=\"12\"/>\n"
            "  </tangibles>\n"
            "  <author name=\"TestAuthor\" />\n"
            "  <patch name=\"VolumePatch\" />\n"
            "</reactablepatch>\n";

        Scene loaded_scene;
        ReactablePatchMetadata metadata;
        std::string error;
        const bool ok = rectai::LoadReactablePatchFromString(
            kRtpVolume, loaded_scene, &metadata, &error);
        assert(ok);
        assert(error.empty());

        assert(loaded_scene.modules().size() == 1U);
        const auto& modules = loaded_scene.modules();
        const auto it = modules.find("1");
        assert(it != modules.end());
        const auto* module = it->second.get();
        const auto* volModule =
            dynamic_cast<const VolumeModule*>(module);
        assert(volModule != nullptr);

        const float volumeParamFromRtp =
            volModule->GetParameterOrDefault("volume", 0.0F);
        assert(volumeParamFromRtp > 0.89F && volumeParamFromRtp < 0.91F);
    }

    // FilterModule envelope initialization test.
    {
        FilterModule filter("filter_test");
        const auto& env = filter.envelope();
        
        // Verify default envelope values (attack, decay, duration, release in ms).
        assert(env.attack == 500.0F);
        assert(env.decay == 500.0F);
        assert(env.duration == 1000.0F);
        assert(env.release == 500.0F);

        // Verify that envelope parameters can be retrieved via GetParameterOrDefault.
        assert(filter.GetParameterOrDefault("attack", 0.0F) == 500.0F);
        assert(filter.GetParameterOrDefault("decay", 0.0F) == 500.0F);
        assert(filter.GetParameterOrDefault("duration", 0.0F) == 1000.0F);
        assert(filter.GetParameterOrDefault("release", 0.0F) == 500.0F);

        // Test envelope setters: modify values and verify they're updated.
        filter.set_envelope_attack(300.0F);
        assert(filter.envelope().attack == 300.0F);
        assert(filter.GetParameterOrDefault("attack", 0.0F) == 300.0F);

        filter.set_envelope_decay(400.0F);
        assert(filter.envelope().decay == 400.0F);
        assert(filter.GetParameterOrDefault("decay", 0.0F) == 400.0F);

        filter.set_envelope_duration(2000.0F);
        assert(filter.envelope().duration == 2000.0F);
        assert(filter.GetParameterOrDefault("duration", 0.0F) == 2000.0F);

        filter.set_envelope_release(600.0F);
        assert(filter.envelope().release == 600.0F);
        assert(filter.GetParameterOrDefault("release", 0.0F) == 600.0F);
    }

    // Default Reactable patch configuration (default.rtp).
    {
        // Locate the bundled default.rtp file under com.reactable/Resources/
        // by probing a small set of likely relative paths (no JUCE here).
        const char* candidates[] = {
            "com.reactable/Resources/default.rtp",
            "./com.reactable/Resources/default.rtp",
            "../com.reactable/Resources/default.rtp",
            "../../com.reactable/Resources/default.rtp",
            "Resources/default.rtp",
            nullptr
        };

        std::string foundPath;
        for (int i = 0; candidates[i] != nullptr; ++i) {
            std::ifstream in(candidates[i]);
            if (in.is_open()) {
                foundPath = candidates[i];
                break;
            }
        }
        assert(!foundPath.empty());

        Scene loaded_scene;
        ReactablePatchMetadata metadata;
        std::string error;

        const bool ok = rectai::LoadReactablePatchFromFile(foundPath,
            loaded_scene, &metadata, &error);
        assert(ok);
        assert(error.empty());

        // The default patch should contain at least the core modules:
        // Output (-1), Volume (1), Tempo (2), Oscillator (46) and
        // Sampleplay banks (48, 49).
        const auto& modules = loaded_scene.modules();
        assert(modules.find("-1") != modules.end());  // Output/master
        assert(modules.find("1") != modules.end());   // Volume
        assert(modules.find("2") != modules.end());   // Tempo
        assert(modules.find("46") != modules.end());  // Oscillator (sine)

        // Master output should not start muted.
        assert(!metadata.master_muted);

        // Volume from default.rtp (volume="90") must be normalised
        // to ~= 0.9 in the internal model.
        const auto volIt = modules.find("1");
        assert(volIt != modules.end());
        const auto* volModule =
            dynamic_cast<const VolumeModule*>(volIt->second.get());
        assert(volModule != nullptr);
        const float volumeParamFromRtp =
            volModule->GetParameterOrDefault("volume", 0.0F);
        assert(volumeParamFromRtp > 0.89F &&
               volumeParamFromRtp < 0.91F);

        // The Oscillator used in the dock (id 46) should have a
        // default gain > 0 so that, once placed on the table, it can
        // produce audible sound without extra interaction.
        const auto oscIt = modules.find("46");
        assert(oscIt != modules.end());
        const auto* oscModule =
            dynamic_cast<const OscillatorModule*>(oscIt->second.get());
        assert(oscModule != nullptr);
        const float oscGain =
            oscModule->GetParameterOrDefault("gain", 0.0F);
        assert(oscGain > 0.0F);

        // Master routing from the Oscillator (46) to Output (-1) may
        // be direct or indirectly chained through filters depending on
        // the current auto-wiring rules. At least one effective route
        // (direct or via a chain) must exist so the oscillator can
        // reach the master output, but the exact topology is not
        // asserted here.
        const auto& connections = loaded_scene.connections();
        assert(!connections.empty());
    }

    std::cout << "rectai-core-tests: OK" << std::endl;
    return 0;
}
