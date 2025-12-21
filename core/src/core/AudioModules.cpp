#include "core/AudioModules.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>

namespace rectai {

namespace {
constexpr uint32_t MakeColour(const uint8_t r, const uint8_t g,
                              const uint8_t b)
{
  return static_cast<uint32_t>(0xFF) << 24U |
         (static_cast<uint32_t>(r) << 16U) |
         (static_cast<uint32_t>(g) << 8U) |
         static_cast<uint32_t>(b);
}
}  // namespace

OscillatorModule::OscillatorModule(const std::string& id,
                                   const float default_freq,
                                   const float default_gain)
    : AudioModuleWithEnvelope(id, ModuleType::kGenerator,
                              /*produces_audio=*/true,
                              /*consumes_audio=*/false,
                              /*produces_midi=*/false,
                              /*consumes_midi=*/true)
{
  set_colour(MakeColour(0x23, 0x66, 0xE1));
  set_label("Oscillator");
    set_description(
      "Tone generator feeding the master bus or downstream modules.");
  // Initialise the mode system to the default (sine waveform),
  set_mode("sine");
  enable_pitch_control(true);
  enable_gain_control(true);
  // Choose a musical frequency range that roughly aligns with the
  // MIDI note window driven by the pitch UI and Sequencer
  // (approximately C2 up to around C8). Values outside this range
  // are clamped by the normalised `freq` parameter.
  set_frequency_mapping(20.0, 3918.0);  // ≈20 Hz – 3.9 kHz
  set_level_mapping(0.02F, 0.18F);
  set_connection_targets({ModuleType::kAudio, ModuleType::kFilter});

  // MIDI note input (from Sequencer or similar) and audio output.
  AddInputPort("in", PortSignalKind::kMidi);
  AddOutputPort("out", PortSignalKind::kAudio);
  SetParameter("freq", default_freq);
  SetParameter("gain", default_gain);
}

float OscillatorModule::default_parameter_value(
    const std::string& name) const
{
  if (name == "freq" || name == "gain") {
    return 0.5F;
  }
  // Delegate remaining parameters (including ADSR envelope when present)
  // to the shared envelope-aware base implementation.
  return AudioModuleWithEnvelope::default_parameter_value(name);
}

const AudioModuleModes& OscillatorModule::supported_modes() const
{
  // Waveforms are exposed to the UI in the same order as the
  // underlying enum so that indices can be mapped directly.
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "sine", "oscillator_sine"},
      AudioModuleMode{1, "saw", "oscillator_saw"},
      AudioModuleMode{2, "square", "oscillator_square"},
      AudioModuleMode{3, "noise", "oscillator_noise"},
  };
  return kModes;
}

void OscillatorModule::on_mode_changed(const int newIndex,
                                       const AudioModuleMode& mode)
{
  // Reuse the base behaviour to keep the icon in sync with the
  // selected mode. At present the audio engine derives the
  // oscillator waveform directly from current_mode_index(), so no
  // additional internal state is required here.
  AudioModule::on_mode_changed(newIndex, mode);
}

FilterModule::FilterModule(const std::string& id,
                           const float default_cutoff,
                           const float default_q)
    : AudioModuleWithEnvelope(id, ModuleType::kFilter,
                              /*produces_audio=*/true,
                              /*consumes_audio=*/true)
{
  set_colour(MakeColour(0x40, 0xE0, 0xA0));
  set_label("Filter");
  set_description("Shapes incoming audio based on spatial relationships.");
  set_mode("bandpass");
  enable_frequency_control(true);
  enable_gain_control(true);
  set_frequency_mapping(100.0, 1900.0);
  set_level_mapping(0.02F, 0.90F);
  allow_any_connection_target();

  AddInputPort("in", PortSignalKind::kAudio);
  AddOutputPort("out", PortSignalKind::kAudio);
  SetParameter("freq", default_cutoff);
  SetParameter("q", default_q);

  initialise_envelope_parameters(
    /*attack_ms=*/500.0F,
    /*decay_ms=*/500.0F,
    /*duration_ms=*/1000.0F,
    /*release_ms=*/500.0F);
}

float FilterModule::default_parameter_value(const std::string& name) const
{
  if (name == "freq" || name == "q") {
    return 0.5F;
  }
  // Delegate ADSR-related defaults to the shared envelope base class.
  return AudioModuleWithEnvelope::default_parameter_value(name);
}

std::optional<AudioModule::XYControlMapping>
FilterModule::xy_control_mapping() const
{
  return AudioModule::XYControlMapping{"freq", "q"};
}

const AudioModuleModes& FilterModule::supported_modes() const
{
  // Filter response types are exposed to the UI in the same order as
  // the internal Mode enum so that indices can be mapped directly.
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "lowpass", "filter_lowpass"},
      AudioModuleMode{1, "bandpass", "filter_bandpass"},
      AudioModuleMode{2, "highpass", "filter_hipass"},
  };
  return kModes;
}

int FilterModule::default_mode_index() const
{
  // The default constructor initialises the filter to band-pass.
  return 1;
}

void FilterModule::on_mode_changed(const int newIndex,
                                   const AudioModuleMode& mode)
{
  // For now FilterModule does not keep additional per-mode internal
  // state beyond what the audio engine derives from the mode id, so
  // we simply reuse the base implementation to update the icon.
  AudioModule::on_mode_changed(newIndex, mode);
}

const AudioModule::SettingsTabs& FilterModule::supported_settings_tabs() const
{
  using SettingsTabs = AudioModule::SettingsTabs;
  using SettingsTabDescriptor = AudioModule::SettingsTabDescriptor;

  static const SettingsTabs kTabs = {
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kXYControl,
                            "tab_2d"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kEnvelope,
                            "tab_envelope"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kSettings,
                            "tab_settings"},
  };
  return kTabs;
}

OutputModule::OutputModule(const std::string& id)
    : AudioModule(id, ModuleType::kAudio,
                  /*produces_audio=*/true, /*consumes_audio=*/true)
{
  set_colour(MakeColour(0xFF, 0xFF, 0xFF));
  set_label("Output");
  set_description("Master output of the scene.");
  set_icon_id("output");
  enable_gain_control(true);
  set_level_mapping(0.02F, 0.98F);
  allow_any_connection_target();

  AddInputPort("in", PortSignalKind::kAudio);
  AddOutputPort("out", PortSignalKind::kAudio);
}

TonalizerModule::TonalizerModule(const std::string& id)
    : AudioModule(id, ModuleType::kSettings,
                  /*produces_audio=*/false, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0xD5, 0xD5, 0xD5));
  set_label("Tonalizer");
  set_description("Quantizes notes to a given scale.");
  set_icon_id("tonalizer");
}

const AudioModuleModes& TonalizerModule::supported_modes() const
{
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "tonalizer", "tonalizer_tonalizer"},
  };
  return kModes;
}

VolumeModule::VolumeModule(const std::string& id)
    : AudioModule(id, ModuleType::kSettings,
                  /*produces_audio=*/false, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0xD5, 0xD5, 0xD5));
  set_label("Volume");
  set_description("Global volume and dynamics control.");
  set_mode("volume");
  enable_gain_control(true);
  set_level_mapping(0.0F, 1.0F);

  SetParameter("volume", 0.9F);
  SetParameter("compression_level", 0.0F);
  SetParameter("compression_on", 0.0F);
  SetParameter("reverb_level", 0.0F);
  SetParameter("reverb_input", 0.0F);
  SetParameter("reverb_on", 0.0F);
  SetParameter("delay_fb", 0.0F);
  SetParameter("delay_time", 0.7F);
}

const AudioModuleModes& VolumeModule::supported_modes() const
{
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "volume", "volume_volume"},
      AudioModuleMode{1, "compressor", "volume_compressor"},
      AudioModuleMode{2, "reverb", "volume_reverb"},
  };
  return kModes;
}

TempoModule::TempoModule(const std::string& id)
    : AudioModule(id, ModuleType::kSettings,
                  /*produces_audio=*/false, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0xD5, 0xD5, 0xD5));
  set_label("Tempo");
  set_description("Global tempo and meter settings.");
  set_mode("tempo");

  SetParameter("tempo", 128.0F);
  SetParameter("meter", 4.0F);
  SetParameter("swing", 0.0F);
}

const AudioModuleModes& TempoModule::supported_modes() const
{
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "tempo", "tempo_tempo"},
      AudioModuleMode{1, "background", "tempo_background"},
  };
  return kModes;
}

const TempoModule::BpmPresetList& TempoModule::bpm_presets()
{
  static const BpmPresetList kPresets = {{
      {"Grave", 40.0F},
      {"Lento", 50.0F},
      {"Larghetto", 63.0F},
      {"Adagio", 73.0F},
      {"Moderato", 95.0F},
      {"House", 125.0F},
      {"Allegro", 139.0F},
      {"Presto", 184.0F},
      {"Prestissimo", 250.0F},
  }};

  return kPresets;
}

const AudioModule::SettingsTabs& TempoModule::supported_settings_tabs() const
{
  using SettingsTabs = AudioModule::SettingsTabs;
  using SettingsTabDescriptor = AudioModule::SettingsTabDescriptor;

  // Single settings tab using a TextScroll-based BPM preset view,
  // rendered with the "tab_bars" icon from the atlas.
  static const SettingsTabs kTabs = {
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kSettings,
                            "tempo_tempo"},
  };
  return kTabs;
}

AccelerometerModule::AccelerometerModule(const std::string& id)
    : AudioModule(id, ModuleType::kSettings,
                  /*produces_audio=*/false, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0xFF, 0xFF, 0x00));
  set_label("Accelerometer");
  set_description("Modulation based on accelerometer-like input.");
  set_icon_id("accelerometer");

  SetParameter("amp_mult", 1.0F);
  SetParameter("freq_mult", 1.0F);
  SetParameter("freq", 12.0F);
  SetParameter("duration", 0.75F);
}

LfoModule::LfoModule(const std::string& id)
    : AudioModule(id, ModuleType::kGenerator,
                  /*produces_audio=*/false, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0x00, 0x00, 0x00));
  set_label("LFO");
  set_description("Low-frequency modulation source.");
  set_mode("sine");
  set_icon_id("lfo");

  AddOutputPort("out", PortSignalKind::kControl);

  SetParameter("freq", 9.0F);
  SetParameter("mult", 0.906058F);
  SetParameter("samplehold", 1.0F);
}

const AudioModuleModes& LfoModule::supported_modes() const
{
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "sine", "lfo_sine"},
      AudioModuleMode{1, "saw", "lfo_saw"},
      AudioModuleMode{2, "square", "lfo_square"},
      AudioModuleMode{3, "noise", "lfo_noise"},
  };
  return kModes;
}

const AudioModule::SettingsTabs& LfoModule::supported_settings_tabs() const
{
  using SettingsTabs = AudioModule::SettingsTabs;
  using SettingsTabDescriptor = AudioModule::SettingsTabDescriptor;

  static const SettingsTabs kTabs = {
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kXYControl,
                            "tab_2d"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kSettings,
                            "tab_settings"},
  };
  return kTabs;
}

std::optional<AudioModule::XYControlMapping>
LfoModule::xy_control_mapping() const
{
  return AudioModule::XYControlMapping{"freq", "mult"};
}

SequencerModule::SequencerModule(const std::string& id)
    : AudioModule(id, ModuleType::kSequencer,
                  /*produces_audio=*/false, /*consumes_audio=*/false,
                  /*produces_midi=*/true, /*consumes_midi=*/false)
{
  set_colour(MakeColour(0x00, 0x00, 0x00));
  set_label("Sequencer");
  set_description("Step sequencer driving notes or triggers.");
  set_mode("sequencer");

  AddOutputPort("out", PortSignalKind::kMidi);

  SetParameter("current_track", 0.0F);
  SetParameter("autoseq_on", 0.0F);
  SetParameter("noteedit_on", 0.0F);
  SetParameter("duration", 1.0F);
  SetParameter("num_tracks", 6.0F);
  SetParameter("offset", 0.0F);

  // Demo preset: simple C major scale (C4..B4) along the first
  // seven steps. This provides an audible example when no
  // SequenceTrack data from .rtp has been loaded; loaders that
  // populate tracks will later call SyncPresetsFromTracks and can
  // overwrite this content.
  static constexpr int kScaleSize = 7;
  const int scale[kScaleSize] = {60, 62, 64, 65, 67, 69, 71};

  SequencerPreset& demoPreset = presets_[0];
  for (int i = 0; i < SequencerPreset::kNumSteps; ++i) {
    SequencerStep& step =
        demoPreset.steps[static_cast<std::size_t>(i)];
    if (i < kScaleSize) {
      step.enabled = true;
      step.velocity01 = 1.0F;
      step.pitch = scale[i];
    } else {
      step.enabled = false;
      step.velocity01 = 0.0F;
      step.pitch = 60;
    }
    // Keep the polyphonic-friendly container in sync with the
    // primary monophonic pitch. Disabled steps expose no pitches.
    step.pitches.clear();
    if (step.enabled) {
      step.pitches.push_back(step.pitch);
    }
  }
}

const AudioModuleModes& SequencerModule::supported_modes() const
{
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "sequencer", "sequencer_sequencer"},
      AudioModuleMode{1, "tenori", "sequencer_tenori"},
      AudioModuleMode{2, "random", "sequencer_random"},
  };
  return kModes;
}

void SequencerModule::SyncPresetsFromTracks()
{
  const std::size_t maxPresets = static_cast<std::size_t>(kNumPresets);
  const std::size_t trackCount = std::min(tracks_.size(), maxPresets);

  for (std::size_t presetIndex = 0; presetIndex < trackCount; ++presetIndex) {
    const SequenceTrack& track = tracks_[presetIndex];
    SequencerPreset& preset = presets_[presetIndex];

    for (int stepIndex = 0; stepIndex < SequencerPreset::kNumSteps;
         ++stepIndex) {
      SequencerStep& step =
          preset.steps[static_cast<std::size_t>(stepIndex)];

      // Enabled flag from steps vector (0/1 per step), when available.
      if (stepIndex < static_cast<int>(track.steps.size())) {
        step.enabled = (track.steps[static_cast<std::size_t>(stepIndex)] != 0);
      } else {
        step.enabled = false;
      }

      // Velocity from volumes vector when present, clamped to [0,1].
      // Disabled steps must always expose zero velocity so that they
      // behave as silent pulses, regardless of any volume encoded in
      // the source track.
      if (stepIndex < static_cast<int>(track.volumes.size())) {
        float v = track.volumes[static_cast<std::size_t>(stepIndex)];
        if (v < 0.0F) {
          v = 0.0F;
        } else if (v > 1.0F) {
          v = 1.0F;
        }
        step.velocity01 = step.enabled ? v : 0.0F;
      } else {
        // When no explicit volume is given, enabled steps default to 1
        // and disabled steps to 0.
        step.velocity01 = step.enabled ? 1.0F : 0.0F;
      }

      // Pitch handling depends on the Sequencer version. In version 1
      // the module behaves as a pulse generator and does not encode
      // melodic information; all active steps use a fixed MIDI note
      // (C4) so that downstream modules can trigger envelopes or
      // gates without caring about pitch. From version 2 onwards we
      // derive the pitch from per-step frequencies when available.

      int pitch = 60;  // Default to C4 for pulse-only mode.
      if (version_ >= 2) {
        if (stepIndex < static_cast<int>(track.step_frequencies.size())) {
          const float freq =
              track.step_frequencies[static_cast<std::size_t>(stepIndex)];
          if (freq > 0.0F) {
            const double ratio = static_cast<double>(freq) / 440.0;
            if (ratio > 0.0) {
              const double midi =
                  69.0 + 12.0 * std::log(ratio) / std::log(2.0);
              pitch = static_cast<int>(std::lround(midi));
            }
          }
        }

        // Clamp to a sensible MIDI range when deriving notes.
        if (pitch < 0) {
          pitch = 0;
        } else if (pitch > 127) {
          pitch = 127;
        }
      }

      step.pitch = pitch;

      // For now, represent the step's pitch as a single-entry
      // polyphonic list when enabled, so future runtimes can
      // iterate `pitches` directly without changing how presets
      // are loaded from .rtp tracks.
      step.pitches.clear();
      if (step.enabled) {
        step.pitches.push_back(step.pitch);
      }
    }
  }
}

DelayModule::DelayModule(const std::string& id)
  : AudioModuleWithEnvelope(id, ModuleType::kAudio,
                /*produces_audio=*/true,
                /*consumes_audio=*/true)
{
  set_colour(MakeColour(0x00, 0x00, 0x00));
  set_label("Delay");
  set_description("Delay / echo effect.");
  set_mode("feedback");

  AddInputPort("in", PortSignalKind::kAudio);
  AddOutputPort("out", PortSignalKind::kAudio);

  SetParameter("delay", 0.66F);
  SetParameter("fb", 0.5F);
  SetParameter("sweep", 0.0F);
}

const AudioModuleModes& DelayModule::supported_modes() const
{
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "feedback", "delay_feedback"},
      AudioModuleMode{1, "reverb", "delay_reverb"},
      // AudioModuleMode{2, "pingpong", "delay_pingpong"},
  };
  return kModes;
}

std::optional<AudioModule::XYControlMapping>
DelayModule::xy_control_mapping() const
{
  return AudioModule::XYControlMapping{"delay", "fb"};
}

const AudioModule::SettingsTabs& DelayModule::supported_settings_tabs() const
{
  using SettingsTabs = AudioModule::SettingsTabs;
  using SettingsTabDescriptor = AudioModule::SettingsTabDescriptor;

  static const SettingsTabs kTabs = {
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kXYControl,
                            "tab_2d"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kEnvelope,
                            "tab_envelope"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kSettings,
                            "tab_settings"},
  };
  return kTabs;
}

ModulatorModule::ModulatorModule(const std::string& id)
  : AudioModuleWithEnvelope(id, ModuleType::kAudio,
                /*produces_audio=*/true,
                /*consumes_audio=*/true)
{
  set_colour(MakeColour(0x00, 0x00, 0x00));
  set_label("Modulator");
  set_description("Ringmod / modulation effect.");
  set_mode("ringmod");

  AddInputPort("in", PortSignalKind::kAudio);
  AddOutputPort("out", PortSignalKind::kAudio);

  SetParameter("effect", 0.5F);
  SetParameter("drywet", 0.5F);
  SetParameter("depth", 0.5F);
  SetParameter("min", 0.5F);
  SetParameter("fb", 0.5F);
}

const AudioModuleModes& ModulatorModule::supported_modes() const
{
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "ringmod", "modulator_ringmod"},
      AudioModuleMode{1, "chorus", "modulator_chorus"},
      AudioModuleMode{2, "flanger", "modulator_flanger"},
  };
  return kModes;
}

std::optional<AudioModule::XYControlMapping>
ModulatorModule::xy_control_mapping() const
{
  return AudioModule::XYControlMapping{"effect", "depth"};
}

const AudioModule::SettingsTabs& ModulatorModule::supported_settings_tabs() const
{
  using SettingsTabs = AudioModule::SettingsTabs;
  using SettingsTabDescriptor = AudioModule::SettingsTabDescriptor;

  static const SettingsTabs kTabs = {
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kXYControl,
                            "tab_2d"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kEnvelope,
                            "tab_envelope"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kSettings,
                            "tab_settings"},
  };
  return kTabs;
}

WaveShaperModule::WaveShaperModule(const std::string& id)
  : AudioModuleWithEnvelope(id, ModuleType::kAudio,
                /*produces_audio=*/true,
                /*consumes_audio=*/true)
{
  set_colour(MakeColour(0x00, 0x00, 0x00));
  set_label("WaveShaper");
  set_description("Waveshaping / distortion effect.");
  set_mode("distort");

  AddInputPort("in", PortSignalKind::kAudio);
  AddOutputPort("out", PortSignalKind::kAudio);

  SetParameter("effect", 0.5F);
  SetParameter("drywet", 0.5F);
}

const AudioModuleModes& WaveShaperModule::supported_modes() const
{
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "distort", "waveshaper_distort"},
      AudioModuleMode{1, "compress", "waveshaper_compress"},
      AudioModuleMode{2, "resample", "waveshaper_resample"},
  };
  return kModes;
}

std::optional<AudioModule::XYControlMapping>
WaveShaperModule::xy_control_mapping() const
{
  return AudioModule::XYControlMapping{"effect", "drywet"};
}

const AudioModule::SettingsTabs& WaveShaperModule::supported_settings_tabs() const
{
  using SettingsTabs = AudioModule::SettingsTabs;
  using SettingsTabDescriptor = AudioModule::SettingsTabDescriptor;

  static const SettingsTabs kTabs = {
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kXYControl,
                            "tab_2d"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kEnvelope,
                            "tab_envelope"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kSettings,
                            "tab_settings"},
  };
  return kTabs;
}

InputModule::InputModule(const std::string& id)
  : AudioModuleWithEnvelope(id, ModuleType::kAudio,
                /*produces_audio=*/true,
                /*consumes_audio=*/false)
{
  set_colour(MakeColour(0x23, 0xCB, 0x43));
  set_label("Input");
  set_description("External audio input.");
  set_icon_id("input");

  AddOutputPort("out", PortSignalKind::kAudio);
  SetParameter("amp", 0.0F);
}

LoopModule::LoopModule(const std::string& id)
  : AudioModuleWithEnvelope(id, ModuleType::kAudio,
                /*produces_audio=*/true,
                /*consumes_audio=*/false)
{
  set_colour(MakeColour(0xD4, 0xCD, 0x06));
  set_label("Loop");
  set_description("Loop player.");
  set_mode("loop");

  AddOutputPort("out", PortSignalKind::kAudio);

  // Expose a dedicated gain control so that the right-hand
  // arc in the UI can drive the Loop output level via the
  // "amp" parameter, matching the behaviour of Sampleplay
  // and Input modules.
  enable_gain_control(true);
  set_level_mapping(0.0F, 1.0F);

  SetParameter("amp", 1.0F);
  SetParameter("sample", 0.0F);
  SetParameter("speed", 1.0F);
}

const AudioModuleModes& LoopModule::supported_modes() const
{
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "loop", "loop_loop"},
      AudioModuleMode{1, "timestretch", "loop_timestretch"},
      // AudioModuleMode{2, "oneshot", "loop_oneshot"},
  };
  return kModes;
}

const AudioModule::SettingsTabs& LoopModule::supported_settings_tabs() const
{
  using SettingsTabs = AudioModule::SettingsTabs;
  using SettingsTabDescriptor = AudioModule::SettingsTabDescriptor;

  static const SettingsTabs kTabs = {
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kLoopFiles,
                            "tab_file"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kEnvelope,
                            "tab_envelope"},
      SettingsTabDescriptor{AudioModule::SettingsTabKind::kSettings,
                            "tab_settings"},
  };
  return kTabs;
}

SampleplayModule::SampleplayModule(const std::string& id)
    : AudioModule(id, ModuleType::kAudio,
          /*produces_audio=*/true, /*consumes_audio=*/false,
          /*produces_midi=*/false, /*consumes_midi=*/true)
{
  set_colour(MakeColour(0x19, 0xCC, 0xDC));
  set_label("Sampleplay");
  set_description("Sample-based playback.");
  set_mode("synth");

  // Expose a gain control so the right-hand arc in the UI can
  // drive the Sampleplay output level. Use a wider level mapping
  // than the default so that SoundFont-based instruments have a
  // comfortable loudness range in combination with the global
  // volume curve.
  enable_pitch_control(true);
  enable_gain_control(true);
  set_level_mapping(0.05F, 0.95F);

  // MIDI input controlling which notes to trigger, and audio output
  // driven by the internal SoundFont engine.
  AddInputPort("in", PortSignalKind::kMidi);
  AddOutputPort("out", PortSignalKind::kAudio);

  SetParameter("amp", 1.0F);
  SetParameter("midifreq", 57.0F);
  SetParameter("channel", 0.0F);
  // The actual filename and instruments are kept in the
  // SampleplayModule-specific data structures.
  active_instrument_index_ = -1;
}

const AudioModuleModes& SampleplayModule::supported_modes() const
{
  static const AudioModuleModes kModes = {
      AudioModuleMode{0, "drum", "sampleplay_drum"},
      AudioModuleMode{1, "synth", "sampleplay_synth"},
      AudioModuleMode{2, "sampler", "sampleplay_sampler"},
  };
  return kModes;
}

bool SampleplayModule::LoadSoundfont(const std::string& path,
                                     std::string* const error_message)
{
  if (error_message != nullptr) {
    *error_message = {};
  }

  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    if (error_message != nullptr) {
      *error_message = "Failed to open soundfont file";
    }
    soundfont_loaded_ = false;
    soundfont_path_.clear();
    return false;
  }

  char header[12]{};
  if (!in.read(header, static_cast<std::streamsize>(sizeof(header)))) {
    if (error_message != nullptr) {
      *error_message = "Soundfont file is too small";
    }
    soundfont_loaded_ = false;
    soundfont_path_.clear();
    return false;
  }

  // Basic SF2 validation: RIFF container with "sfbk" form type.
  const bool has_riff = std::memcmp(header, "RIFF", 4) == 0;
  const bool has_sfbk = std::memcmp(header + 8, "sfbk", 4) == 0;
  if (!has_riff || !has_sfbk) {
    if (error_message != nullptr) {
      *error_message = "File does not look like a valid SF2 soundfont";
    }
    soundfont_loaded_ = false;
    soundfont_path_.clear();
    return false;
  }

  soundfont_path_ = path;
  soundfont_loaded_ = true;
  return true;
}

int SampleplayModule::active_instrument_index() const
{
  if (instruments_.empty()) {
    return -1;
  }

  if (active_instrument_index_ < 0) {
    return 0;
  }

  const int maxIndex =
      static_cast<int>(instruments_.size()) - 1;
  if (active_instrument_index_ > maxIndex) {
    return maxIndex;
  }

  return active_instrument_index_;
}

void SampleplayModule::set_active_instrument_index(const int index)
{
  if (instruments_.empty()) {
    active_instrument_index_ = -1;
    return;
  }

  const int maxIndex =
      static_cast<int>(instruments_.size()) - 1;
  if (index < 0) {
    active_instrument_index_ = 0;
  } else if (index > maxIndex) {
    active_instrument_index_ = maxIndex;
  } else {
    active_instrument_index_ = index;
  }
}

void SampleplayModule::CycleInstrument()
{
  if (instruments_.empty()) {
    active_instrument_index_ = -1;
    return;
  }

  const int count = static_cast<int>(instruments_.size());
  int index = active_instrument_index_;
  if (index < 0 || index >= count) {
    index = 0;
  }

  index = (index + 1) % count;
  active_instrument_index_ = index;
}

const SampleInstrument* SampleplayModule::active_instrument() const
{
  if (instruments_.empty()) {
    return nullptr;
  }

  const int index = active_instrument_index();
  if (index < 0 ||
      index >= static_cast<int>(instruments_.size())) {
    return nullptr;
  }

  return &instruments_[static_cast<std::size_t>(index)];
}

void SampleplayModule::CycleBank()
{
  if (instruments_.empty() || default_preset_indices_.empty()) {
    return;
  }

  const int bankCount =
      static_cast<int>(default_preset_indices_.size());
  if (bankCount <= 0) {
    return;
  }

  // Advance the logical channel within the range of banks defined by
  // the .rtp (indices of <instrument>) and select the first bank that
  // has a valid preset index.
  for (int step = 0; step < bankCount; ++step) {
    const int nextChannel = (channel_ + 1 + step) % bankCount;
    const int preferredIndex =
        default_preset_indices_[static_cast<std::size_t>(nextChannel)];
    if (preferredIndex >= 0 &&
        preferredIndex < static_cast<int>(instruments_.size())) {
      channel_ = nextChannel;
      set_active_instrument_index(preferredIndex);
      return;
    }
  }
}

}  // namespace rectai
