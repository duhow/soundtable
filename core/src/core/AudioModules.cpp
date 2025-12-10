#include "core/AudioModules.h"

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
    : AudioModule(id, ModuleType::kGenerator,
                  /*produces_audio=*/true, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0x20, 0x90, 0xFF));
  set_label("Oscillator");
  set_description(
      "Tone generator feeding the master bus or downstream modules.");
  set_waveform(Waveform::kSine);
  enable_frequency_control(true);
  enable_gain_control(true);
  set_frequency_mapping(200.0, 800.0);  // 200–1000 Hz
  set_level_mapping(0.02F, 0.18F);
  set_connection_targets({ModuleType::kAudio, ModuleType::kFilter});

  AddOutputPort("out", true);
  SetParameter("freq", default_freq);
  SetParameter("gain", default_gain);
}

float OscillatorModule::default_parameter_value(
    const std::string& name) const
{
  if (name == "freq" || name == "gain") {
    return 0.5F;
  }
  return AudioModule::default_parameter_value(name);
}

void OscillatorModule::set_waveform(const Waveform waveform)
{
  waveform_ = waveform;
  switch (waveform_) {
    case Waveform::kSine:
      set_icon_id("oscillator_sine");
      break;
    case Waveform::kSaw:
      set_icon_id("oscillator_saw");
      break;
    case Waveform::kSquare:
      set_icon_id("oscillator_square");
      break;
    case Waveform::kNoise:
      set_icon_id("oscillator_noise");
      break;
  }
}

void OscillatorModule::set_waveform_from_subtype(
    const std::string& subtype)
{
  if (subtype == "sine") {
    set_waveform(Waveform::kSine);
  } else if (subtype == "saw") {
    set_waveform(Waveform::kSaw);
  } else if (subtype == "square") {
    set_waveform(Waveform::kSquare);
  } else if (subtype == "noise") {
    set_waveform(Waveform::kNoise);
  } else {
    set_waveform(Waveform::kSine);
  }
}

void OscillatorModule::cycle_waveform()
{
  const int current = static_cast<int>(waveform_);
  const int next = (current + 1) % 4;
  set_waveform(static_cast<Waveform>(next));
}

std::string OscillatorModule::subtype_string() const
{
  switch (waveform_) {
    case Waveform::kSine:
      return "sine";
    case Waveform::kSaw:
      return "saw";
    case Waveform::kSquare:
      return "square";
    case Waveform::kNoise:
      return "noise";
  }
  return "sine";
}

FilterModule::FilterModule(const std::string& id,
                           const float default_cutoff,
                           const float default_q)
    : AudioModule(id, ModuleType::kFilter,
                  /*produces_audio=*/true, /*consumes_audio=*/true)
{
  set_colour(MakeColour(0x40, 0xE0, 0xA0));
  set_label("Filter");
  set_description("Shapes incoming audio based on spatial relationships.");
  set_mode(Mode::kBandPass);
  enable_frequency_control(true);
  enable_gain_control(true);
  set_frequency_mapping(200.0, 800.0);
  set_level_mapping(0.02F, 0.18F);
  allow_any_connection_target();

  AddInputPort("in", true);
  AddOutputPort("out", true);
  SetParameter("freq", default_cutoff);
  SetParameter("q", default_q);
}

float FilterModule::default_parameter_value(const std::string& name) const
{
  if (name == "freq" || name == "q") {
    return 0.5F;
  }
  return AudioModule::default_parameter_value(name);
}

void FilterModule::set_mode(const Mode mode)
{
  mode_ = mode;
  switch (mode_) {
    case Mode::kLowPass:
      set_icon_id("filter_lowpass");
      break;
    case Mode::kBandPass:
      set_icon_id("filter_bandpass");
      break;
    case Mode::kHighPass:
      set_icon_id("filter_hipass");
      break;
  }
}

void FilterModule::cycle_mode()
{
  const int current = static_cast<int>(mode_);
  const int next = (current + 1) % 3;
  set_mode(static_cast<Mode>(next));
}

void FilterModule::set_mode_from_subtype(const std::string& subtype)
{
  if (subtype == "lowpass") {
    set_mode(Mode::kLowPass);
  } else if (subtype == "highpass") {
    set_mode(Mode::kHighPass);
  } else if (subtype == "bandpass") {
    set_mode(Mode::kBandPass);
  } else {
    // Keep default (band-pass) for unknown subtypes.
    set_mode(Mode::kBandPass);
  }
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

  AddInputPort("in", true);
  AddOutputPort("out", true);
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

VolumeModule::VolumeModule(const std::string& id)
    : AudioModule(id, ModuleType::kAudio,
                  /*produces_audio=*/true, /*consumes_audio=*/true)
{
  set_colour(MakeColour(0xD5, 0xD5, 0xD5));
  set_label("Volume");
  set_description("Global volume and dynamics control.");
  set_icon_id("volume");
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

TempoModule::TempoModule(const std::string& id)
    : AudioModule(id, ModuleType::kSettings,
                  /*produces_audio=*/false, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0xD5, 0xD5, 0xD5));
  set_label("Tempo");
  set_description("Global tempo and meter settings.");
  set_icon_id("tempo");

  SetParameter("tempo", 128.0F);
  SetParameter("meter", 4.0F);
  SetParameter("swing", 0.0F);
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
  set_icon_id("lfo");

  AddOutputPort("out", true);

  SetParameter("freq", 9.0F);
  SetParameter("mult", 0.906058F);
  SetParameter("samplehold", 1.0F);
}

SequencerModule::SequencerModule(const std::string& id)
    : AudioModule(id, ModuleType::kSequencer,
                  /*produces_audio=*/false, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0x00, 0x00, 0x00));
  set_label("Sequencer");
  set_description("Step sequencer driving notes or triggers.");
  set_icon_id("sequencer");

  AddOutputPort("out", true);

  SetParameter("current_track", 0.0F);
  SetParameter("autoseq_on", 0.0F);
  SetParameter("noteedit_on", 0.0F);
  SetParameter("duration", 1.0F);
  SetParameter("num_tracks", 6.0F);
  SetParameter("offset", 0.0F);
}

DelayModule::DelayModule(const std::string& id)
    : AudioModule(id, ModuleType::kAudio,
                  /*produces_audio=*/true, /*consumes_audio=*/true)
{
  set_colour(MakeColour(0x00, 0x00, 0x00));
  set_label("Delay");
  set_description("Delay / echo effect.");
  set_icon_id("delay");

  AddInputPort("in", true);
  AddOutputPort("out", true);

  SetParameter("delay", 0.66F);
  SetParameter("fb", 0.5F);
  SetParameter("sweep", 0.0F);
}

ModulatorModule::ModulatorModule(const std::string& id)
    : AudioModule(id, ModuleType::kAudio,
                  /*produces_audio=*/true, /*consumes_audio=*/true)
{
  set_colour(MakeColour(0x00, 0x00, 0x00));
  set_label("Modulator");
  set_description("Ringmod / modulation effect.");
  set_icon_id("modulator");

  AddInputPort("in", true);
  AddOutputPort("out", true);

  SetParameter("effect", 0.5F);
  SetParameter("drywet", 0.5F);
  SetParameter("depth", 0.5F);
  SetParameter("min", 0.5F);
  SetParameter("fb", 0.5F);
}

WaveShaperModule::WaveShaperModule(const std::string& id)
    : AudioModule(id, ModuleType::kAudio,
                  /*produces_audio=*/true, /*consumes_audio=*/true)
{
  set_colour(MakeColour(0x00, 0x00, 0x00));
  set_label("WaveShaper");
  set_description("Waveshaping / distortion effect.");
  set_icon_id("waveshaper");

  AddInputPort("in", true);
  AddOutputPort("out", true);

  SetParameter("effect", 0.5F);
  SetParameter("drywet", 0.5F);
}

InputModule::InputModule(const std::string& id)
    : AudioModule(id, ModuleType::kAudio,
                  /*produces_audio=*/true, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0x23, 0xCB, 0x43));
  set_label("Input");
  set_description("External audio input.");
  set_icon_id("input");

  AddOutputPort("out", true);
  SetParameter("amp", 0.0F);
}

LoopModule::LoopModule(const std::string& id)
    : AudioModule(id, ModuleType::kAudio,
                  /*produces_audio=*/true, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0xD4, 0xCD, 0x06));
  set_label("Loop");
  set_description("Loop player.");
  set_icon_id("loop");

  AddOutputPort("out", true);

  SetParameter("amp", 1.0F);
  SetParameter("sample", 0.0F);
  SetParameter("speed", 1.0F);
}

SampleplayModule::SampleplayModule(const std::string& id)
    : AudioModule(id, ModuleType::kAudio,
                  /*produces_audio=*/true, /*consumes_audio=*/false)
{
  set_colour(MakeColour(0x19, 0xCC, 0xDC));
  set_label("Sampleplay");
  set_description("Sample-based playback.");
  set_icon_id("sampleplay");

  AddOutputPort("out", true);

  SetParameter("amp", 1.0F);
  SetParameter("midifreq", 57.0F);
  // The actual filename and instruments are kept in the
  // SampleplayModule-specific data structures.
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

}  // namespace rectai
