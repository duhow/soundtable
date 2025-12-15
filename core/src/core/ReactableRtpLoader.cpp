#include "core/ReactableRtpLoader.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

#include "core/AudioModules.h"

namespace rectai {
namespace {

using AttributeMap = std::unordered_map<std::string, std::string>;

std::string Trim(const std::string& s)
{
  auto begin = s.begin();
  while (begin != s.end() && std::isspace(static_cast<unsigned char>(*begin))) {
    ++begin;
  }
  auto end = s.end();
  do {
    --end;
  } while (end >= begin && std::isspace(static_cast<unsigned char>(*end)));

  return std::string(begin, end + 1);
}

AttributeMap ParseAttributes(const std::string& tag_content)
{
  AttributeMap attrs;
  std::size_t pos = 0;

  while (pos < tag_content.size()) {
    while (pos < tag_content.size() && std::isspace(static_cast<unsigned char>(tag_content[pos]))) {
      ++pos;
    }
    if (pos >= tag_content.size()) {
      break;
    }

    // Read name.
    std::size_t name_start = pos;
    while (pos < tag_content.size() && tag_content[pos] != '=' &&
           !std::isspace(static_cast<unsigned char>(tag_content[pos]))) {
      ++pos;
    }
    if (pos >= tag_content.size() || tag_content[pos] != '=') {
      // Skip token that is not an attribute (e.g. tag name itself).
      while (pos < tag_content.size() && !std::isspace(static_cast<unsigned char>(tag_content[pos]))) {
        ++pos;
      }
      continue;
    }

    std::string name = tag_content.substr(name_start, pos - name_start);

    // Skip '='.
    ++pos;
    while (pos < tag_content.size() && std::isspace(static_cast<unsigned char>(tag_content[pos]))) {
      ++pos;
    }
    if (pos >= tag_content.size() || tag_content[pos] != '"') {
      continue;
    }

    // Read quoted value.
    ++pos;  // skip opening quote
    std::size_t value_start = pos;
    while (pos < tag_content.size() && tag_content[pos] != '"') {
      ++pos;
    }
    if (pos > value_start) {
      std::string value = tag_content.substr(value_start, pos - value_start);
      attrs[name] = value;
    }
    if (pos < tag_content.size() && tag_content[pos] == '"') {
      ++pos;  // skip closing quote
    }
  }

  return attrs;
}

float ParseFloat(const AttributeMap& attrs, const std::string& key,
                 const float default_value)
{
  const auto it = attrs.find(key);
  if (it == attrs.end()) {
    return default_value;
  }
  try {
    return std::stof(it->second);
  } catch (...) {
    return default_value;
  }
}

int ParseInt(const AttributeMap& attrs, const std::string& key,
             const int default_value)
{
  const auto it = attrs.find(key);
  if (it == attrs.end()) {
    return default_value;
  }
  try {
    return std::stoi(it->second);
  } catch (...) {
    return default_value;
  }
}

// Parses boolean-like attributes that are encoded as integers (0/1) in the
// .rtp schema. Any non-zero value is treated as true.
bool ParseBool(const AttributeMap& attrs, const std::string& key,
               const bool default_value)
{
  const auto it = attrs.find(key);
  if (it == attrs.end()) {
    return default_value;
  }
  try {
    const int v = std::stoi(it->second);
    return v != 0;
  } catch (...) {
    return default_value;
  }
}

// Parses a colour attribute encoded as a hexadecimal string. Reactable
// patches typically use 8-digit RGB + alpha (e.g. d5d5d5ff, where the
// last byte is the alpha channel), but some files may provide 6-digit
// RGB. In that case we assume an opaque alpha of 0xFF.
std::uint32_t ParseColourArgb(const AttributeMap& attrs,
                              const std::string& key,
                              const std::uint32_t default_value)
{
  const auto it = attrs.find(key);
  if (it == attrs.end()) {
    return default_value;
  }

  const std::string& s = it->second;
  if (s.empty()) {
    return default_value;
  }

  try {
    const std::uint32_t value =
        static_cast<std::uint32_t>(std::stoul(s, nullptr, 16));

    if (s.size() == 8U) {
      // Interpret as RRGGBBAA (RGB + alpha) and convert to the
      // internal ARGB layout expected by the rest of the code.
      const std::uint32_t rgb = value >> 8U;
      const std::uint32_t alpha = value & 0xFFU;
      return (alpha << 24U) | (rgb & 0x00FFFFFFU);
    }
    if (s.size() == 6U) {
      // Treat as RGB and prepend opaque alpha.
      return (0xFFU << 24U) | (value & 0x00FFFFFFU);
    }
  } catch (...) {
    // Fall through to default.
  }

  return default_value;
}

std::vector<float> SplitFloats(const std::string& csv)
{
  std::vector<float> values;
  std::stringstream ss(csv);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token = Trim(token);
    if (token.empty()) {
      continue;
    }
    try {
      values.push_back(std::stof(token));
    } catch (...) {
      // Ignore malformed entries.
    }
  }
  return values;
}

std::vector<int> SplitInts(const std::string& csv)
{
  std::vector<int> values;
  std::stringstream ss(csv);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token = Trim(token);
    if (token.empty()) {
      continue;
    }
    try {
      values.push_back(std::stoi(token));
    } catch (...) {
      // Ignore malformed entries.
    }
  }
  return values;
}

Envelope ParseEnvelope(const std::string& tag_content)
{
  Envelope env;
  const auto attrs = ParseAttributes(tag_content);
  env.attack = ParseFloat(attrs, "attack", 0.0F);
  env.decay = ParseFloat(attrs, "decay", 0.0F);
  env.duration = ParseFloat(attrs, "duration", 0.0F);
  env.release = ParseFloat(attrs, "release", 0.0F);

  const auto it_x = attrs.find("points_x");
  if (it_x != attrs.end()) {
    env.points_x = SplitFloats(it_x->second);
  }
  const auto it_y = attrs.find("points_y");
  if (it_y != attrs.end()) {
    env.points_y = SplitFloats(it_y->second);
  }
  return env;
}

// Helper to find the next tag with the given name inside [start, end).
std::size_t FindTag(const std::string& xml, const std::string& name,
                    std::size_t start, std::size_t end)
{
  const std::string pattern = "<" + name;
  std::size_t pos = xml.find(pattern, start);
  if (pos == std::string::npos || pos >= end) {
    return std::string::npos;
  }
  return pos;
}

// Extracts the raw tag content (between '<' and '>') starting at position `tag_pos`.
// Returns the index of the closing '>' in `out_end`.
std::string ExtractTagContent(const std::string& xml, std::size_t tag_pos,
                              std::size_t* out_end)
{
  const std::size_t open = xml.find('<', tag_pos);
  if (open == std::string::npos) {
    *out_end = std::string::npos;
    return {};
  }
  const std::size_t close = xml.find('>', open + 1U);
  if (close == std::string::npos) {
    *out_end = std::string::npos;
    return {};
  }
  *out_end = close;
  return xml.substr(open + 1U, close - open - 1U);
}

// Convenience: converts degrees to radians.
float DegreesToRadians(const float degrees)
{
  constexpr float kPi = 3.14159265358979323846F;
  return degrees * (kPi / 180.0F);
}

}  // namespace

bool LoadReactablePatchFromString(const std::string& xml, Scene& scene,
                                  ReactablePatchMetadata* const metadata,
                                  std::string* const error_message)
{
  // Optional metadata: author and patch name.
  if (metadata != nullptr) {
    metadata->author_name.clear();
    metadata->patch_name.clear();
    metadata->master_colour_argb = 0xFFFFFFFFU;
    metadata->master_muted = false;
  }

  // Extract author.
  {
    const std::size_t author_pos = xml.find("<author");
    if (author_pos != std::string::npos) {
      std::size_t end = 0U;
      const auto tag_content = ExtractTagContent(xml, author_pos, &end);
      const auto attrs = ParseAttributes(tag_content);
      const auto it = attrs.find("name");
      if (metadata != nullptr && it != attrs.end()) {
        metadata->author_name = it->second;
      }
    }
  }

  // Extract patch name.
  {
    const std::size_t patch_pos = xml.find("<patch");
    if (patch_pos != std::string::npos) {
      std::size_t end = 0U;
      const auto tag_content = ExtractTagContent(xml, patch_pos, &end);
      const auto attrs = ParseAttributes(tag_content);
      const auto it = attrs.find("name");
      if (metadata != nullptr && it != attrs.end()) {
        metadata->patch_name = it->second;
      }
    }
  }

  // Map from tangible numeric id -> module id string.
  std::unordered_map<int, std::string> tangible_to_module_id;

  // Iterate tangibles.
  std::size_t pos = xml.find("<tangibles");
  if (pos == std::string::npos) {
    // No tangibles: not considered an error, just empty scene.
    return true;
  }

  const std::size_t tangibles_start = xml.find('>', pos);
  if (tangibles_start == std::string::npos) {
    if (error_message != nullptr) {
      *error_message = "Malformed <tangibles> tag";
    }
    return false;
  }

  const std::size_t tangibles_end = xml.find("</tangibles>", tangibles_start);
  if (tangibles_end == std::string::npos) {
    if (error_message != nullptr) {
      *error_message = "Missing </tangibles> closing tag";
    }
    return false;
  }

  pos = xml.find("<tangible", tangibles_start);
  while (pos != std::string::npos && pos < tangibles_end) {
    std::size_t tag_end = 0U;
    const std::string tag_content = ExtractTagContent(xml, pos, &tag_end);
    if (tag_end == std::string::npos) {
      break;
    }

    const auto attrs = ParseAttributes(tag_content);

    const auto it_type = attrs.find("type");
    const auto it_id = attrs.find("id");
    if (it_type == attrs.end() || it_id == attrs.end()) {
      pos = xml.find("<tangible", tag_end);
      continue;
    }

    std::string type = it_type->second;
    int tangible_id = 0;
    try {
      tangible_id = std::stoi(it_id->second);
    } catch (...) {
      tangible_id = 0;
    }

    // Use the numeric id as string for module and logical ids.
    const std::string module_id = it_id->second;
    tangible_to_module_id[tangible_id] = module_id;

    // Determine inner content range for this tangible.
    const bool self_closing = !tag_content.empty() &&
                              tag_content.back() == '/';
    std::size_t inner_start = tag_end + 1U;
    std::size_t inner_end = inner_start;
    if (!self_closing) {
      const std::size_t close_pos = xml.find("</tangible>", tag_end);
      if (close_pos == std::string::npos || close_pos > tangibles_end) {
        if (error_message != nullptr) {
          *error_message = "Malformed <tangible> block";
        }
        return false;
      }
      inner_end = close_pos;
      pos = xml.find("<tangible", close_pos);
    } else {
      pos = xml.find("<tangible", tag_end);
    }

    // Create module according to type.
    std::unique_ptr<AudioModule> module;

    if (type == "Output") {
      module = std::make_unique<OutputModule>(module_id);

      if (metadata != nullptr) {
        metadata->master_muted =
            ParseBool(attrs, "muted", metadata->master_muted);
      }
    } else if (type == "Tonalizer") {
      auto tonalizer = std::make_unique<TonalizerModule>(module_id);
      // Parse <tone> children.
      std::size_t tone_pos = FindTag(xml, "tone", inner_start, inner_end);
      while (tone_pos != std::string::npos) {
        std::size_t tone_end = 0U;
        const auto tone_tag = ExtractTagContent(xml, tone_pos, &tone_end);
        const auto tone_attrs = ParseAttributes(tone_tag);
        ToneDefinition tone_def;
        tone_def.key = ParseInt(tone_attrs, "key", 0);
        const auto it_scale = tone_attrs.find("scale");
        if (it_scale != tone_attrs.end()) {
          tone_def.scale = SplitFloats(it_scale->second);
        }
        tonalizer->mutable_tones().push_back(std::move(tone_def));
        tone_pos = FindTag(xml, "tone", tone_end, inner_end);
      }
      module = std::move(tonalizer);
    } else if (type == "Volume") {
      auto volume = std::make_unique<VolumeModule>(module_id);
      // Copy numeric attributes into parameters.
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point" || key == "subtype") {
          continue;
        }
        try {
          float parsed = std::stof(value);
          // Reactable stores the Volume module level as a
          // percentage (0..100). Normalize it to [0,1] for the
          // internal model when values > 1.0 are detected.
          if (key == "volume" && parsed > 1.0F) {
            parsed = parsed / 100.0F;
          }
          volume->SetParameter(key, parsed);
        } catch (...) {
          // Ignore non-float parameters.
        }
      }
      module = std::move(volume);
    } else if (type == "Tempo") {
      auto tempo = std::make_unique<TempoModule>(module_id);
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point") {
          continue;
        }
        try {
          tempo->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      module = std::move(tempo);
    } else if (type == "Accelerometer") {
      auto accel = std::make_unique<AccelerometerModule>(module_id);
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point") {
          continue;
        }
        try {
          accel->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      module = std::move(accel);
    } else if (type == "LFO") {
      auto lfo = std::make_unique<LfoModule>(module_id);
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point") {
          continue;
        }
        try {
          lfo->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      module = std::move(lfo);
    } else if (type == "Sequencer") {
      auto seq = std::make_unique<SequencerModule>(module_id);
      // Copy tangible-level numeric attributes.
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point" || key == "subtype") {
          continue;
        }
        try {
          seq->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      // Parse <sequence> children into SequenceTrack structures.
      std::size_t seq_pos = FindTag(xml, "sequence", inner_start, inner_end);
      while (seq_pos != std::string::npos) {
        std::size_t seq_end = 0U;
        const auto seq_tag = ExtractTagContent(xml, seq_pos, &seq_end);
        const auto seq_attrs = ParseAttributes(seq_tag);
        SequenceTrack track;
        const auto it_rows = seq_attrs.find("rows");
        if (it_rows != seq_attrs.end()) {
          track.rows = SplitInts(it_rows->second);
        }
        track.speed = ParseInt(seq_attrs, "speed", 0);
        const auto it_speed_type = seq_attrs.find("speed_type");
        if (it_speed_type != seq_attrs.end()) {
          track.speed_type = it_speed_type->second;
        }
        const auto it_step_freq = seq_attrs.find("step_frequencies");
        if (it_step_freq != seq_attrs.end()) {
          track.step_frequencies = SplitFloats(it_step_freq->second);
        }
        const auto it_steps = seq_attrs.find("steps");
        if (it_steps != seq_attrs.end()) {
          track.steps = SplitInts(it_steps->second);
        }
        // tenori layers.
        for (int layer = 0; layer < SequenceTrack::kTenoriLayerCount; ++layer) {
          const std::string key = "tenori" + std::to_string(layer);
          const auto it_layer = seq_attrs.find(key);
          if (it_layer != seq_attrs.end()) {
            track.tenori_layers[static_cast<std::size_t>(layer)] =
                SplitInts(it_layer->second);
          }
        }
        const auto it_volumes = seq_attrs.find("volumes");
        if (it_volumes != seq_attrs.end()) {
          track.volumes = SplitFloats(it_volumes->second);
        }
        seq->mutable_tracks().push_back(std::move(track));
        seq_pos = FindTag(xml, "sequence", seq_end, inner_end);
      }
      // Derive high-level monophonic presets from the loaded
      // SequenceTrack data so that the runtime sequencer can use a
      // fixed-size 6x16 bank even when the original .rtp contains
      // more low-level detail.
      seq->SyncPresetsFromTracks();
      module = std::move(seq);
    } else if (type == "Filter") {
      auto filter = std::make_unique<FilterModule>(module_id);

      const auto itSubtype = attrs.find("subtype");
      if (itSubtype != attrs.end()) {
        filter->set_mode_from_subtype(itSubtype->second);
      }

      // Copy tangible-level numeric attributes.
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point" || key == "subtype") {
          continue;
        }
        try {
          filter->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      // Parse optional <envelope>.
      const std::size_t env_pos = FindTag(xml, "envelope", inner_start, inner_end);
      if (env_pos != std::string::npos) {
        std::size_t env_end = 0U;
        const auto env_tag = ExtractTagContent(xml, env_pos, &env_end);
        filter->mutable_envelope() = ParseEnvelope(env_tag);
      }
      module = std::move(filter);
    } else if (type == "Delay") {
      auto delay = std::make_unique<DelayModule>(module_id);
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point" || key == "subtype") {
          continue;
        }
        try {
          delay->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      const std::size_t env_pos = FindTag(xml, "envelope", inner_start, inner_end);
      if (env_pos != std::string::npos) {
        std::size_t env_end = 0U;
        const auto env_tag = ExtractTagContent(xml, env_pos, &env_end);
        delay->mutable_envelope() = ParseEnvelope(env_tag);
      }
      // hardlink(s)
      std::size_t hl_pos = FindTag(xml, "hardlink", inner_start, inner_end);
      while (hl_pos != std::string::npos) {
        std::size_t hl_end = 0U;
        const auto hl_tag = ExtractTagContent(xml, hl_pos, &hl_end);
        const auto hl_attrs = ParseAttributes(hl_tag);
        const int target_id = ParseInt(hl_attrs, "to", 0);
        if (target_id != 0) {
          delay->mutable_hardlink_targets().push_back(target_id);
        }
        hl_pos = FindTag(xml, "hardlink", hl_end, inner_end);
      }
      module = std::move(delay);
    } else if (type == "Modulator") {
      auto mod = std::make_unique<ModulatorModule>(module_id);
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point" || key == "subtype") {
          continue;
        }
        try {
          mod->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      const std::size_t env_pos = FindTag(xml, "envelope", inner_start, inner_end);
      if (env_pos != std::string::npos) {
        std::size_t env_end = 0U;
        const auto env_tag = ExtractTagContent(xml, env_pos, &env_end);
        mod->mutable_envelope() = ParseEnvelope(env_tag);
      }
      module = std::move(mod);
    } else if (type == "WaveShaper") {
      auto ws = std::make_unique<WaveShaperModule>(module_id);
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point" || key == "subtype") {
          continue;
        }
        try {
          ws->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      const std::size_t env_pos = FindTag(xml, "envelope", inner_start, inner_end);
      if (env_pos != std::string::npos) {
        std::size_t env_end = 0U;
        const auto env_tag = ExtractTagContent(xml, env_pos, &env_end);
        ws->mutable_envelope() = ParseEnvelope(env_tag);
      }
      module = std::move(ws);
    } else if (type == "Input") {
      auto input = std::make_unique<InputModule>(module_id);
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point") {
          continue;
        }
        try {
          input->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      const std::size_t env_pos = FindTag(xml, "envelope", inner_start, inner_end);
      if (env_pos != std::string::npos) {
        std::size_t env_end = 0U;
        const auto env_tag = ExtractTagContent(xml, env_pos, &env_end);
        input->mutable_envelope() = ParseEnvelope(env_tag);
      }
      module = std::move(input);
    } else if (type == "Loop") {
      auto loop = std::make_unique<LoopModule>(module_id);
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point" || key == "subtype") {
          continue;
        }
        try {
          loop->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      const std::size_t env_pos = FindTag(xml, "envelope", inner_start, inner_end);
      if (env_pos != std::string::npos) {
        std::size_t env_end = 0U;
        const auto env_tag = ExtractTagContent(xml, env_pos, &env_end);
        loop->mutable_envelope() = ParseEnvelope(env_tag);
      }
      // Parse <loop> children describing audio loops.
      std::size_t lp_pos = FindTag(xml, "loop", inner_start, inner_end);
      while (lp_pos != std::string::npos) {
        std::size_t lp_end = 0U;
        const auto lp_tag = ExtractTagContent(xml, lp_pos, &lp_end);
        const auto lp_attrs = ParseAttributes(lp_tag);
        LoopDefinition def;
        def.beats = ParseInt(lp_attrs, "beats", 0);
        const auto it_file = lp_attrs.find("filename");
        if (it_file != lp_attrs.end()) {
          def.filename = it_file->second;
        }
        def.order = ParseInt(lp_attrs, "order", 0);
        loop->mutable_loops().push_back(std::move(def));
        lp_pos = FindTag(xml, "loop", lp_end, inner_end);
      }
      module = std::move(loop);
    } else if (type == "Oscillator") {
      auto osc = std::make_unique<OscillatorModule>(module_id);
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point" || key == "subtype") {
          continue;
        }
        try {
          osc->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }
      const auto it_subtype = attrs.find("subtype");
      if (it_subtype != attrs.end()) {
        osc->set_waveform_from_subtype(it_subtype->second);
      }
      const std::size_t env_pos = FindTag(xml, "envelope", inner_start, inner_end);
      if (env_pos != std::string::npos) {
        std::size_t env_end = 0U;
        const auto env_tag = ExtractTagContent(xml, env_pos, &env_end);
        osc->mutable_envelope() = ParseEnvelope(env_tag);
      }
      module = std::move(osc);
    } else if (type == "Sampleplay") {
      auto sp = std::make_unique<SampleplayModule>(module_id);
      for (const auto& [key, value] : attrs) {
        if (key == "type" || key == "id" || key == "x" || key == "y" ||
            key == "angle" || key == "color" || key == "docked" ||
            key == "muted" || key == "point" || key == "subtype" ||
            key == "filename") {
          continue;
        }
        try {
          sp->SetParameter(key, std::stof(value));
        } catch (...) {
        }
      }

      // Logical channel/bank index, used to select between the
      // different logical banks described by the <instrument>
      // children (e.g. drums vs synths).
      sp->set_channel(ParseInt(attrs, "channel", 0));

      // Remember the raw SoundFont filename declared in the patch so
      // the UI can later resolve it against the com.reactable/
      // content tree and call SampleplayModule::LoadSoundfont() with
      // a fully qualified path.
      const auto it_filename = attrs.find("filename");
      if (it_filename != attrs.end() && !it_filename->second.empty()) {
        sp->set_raw_soundfont_name(it_filename->second);
      }

      // Instruments as declared in the .rtp patch. The order of
      // these elements defines the logical banks (index 0 → bank 0,
      // index 1 → bank 1, etc.), which will later be mapped to
      // actual SoundFont presets by name.
      std::size_t inst_pos = FindTag(xml, "instrument", inner_start, inner_end);
      while (inst_pos != std::string::npos) {
        std::size_t inst_end = 0U;
        const auto inst_tag = ExtractTagContent(xml, inst_pos, &inst_end);
        const auto inst_attrs = ParseAttributes(inst_tag);
        const auto it_name = inst_attrs.find("name");
        if (it_name != inst_attrs.end()) {
          SampleInstrument inst;
          inst.name = it_name->second;
          sp->mutable_instruments().push_back(std::move(inst));
        }
        inst_pos = FindTag(xml, "instrument", inst_end, inner_end);
      }

      // Default to the first instrument when available so that the UI
      // has a deterministic starting point for the active instrument
      // title and right-click cycling.
      if (!sp->instruments().empty()) {
        sp->set_active_instrument_index(0);
      }

      module = std::move(sp);
    } else {
      // Unknown or unsupported type for now: skip.
    }

    if (module != nullptr) {
      // If the tangible declares a colour attribute, use it to override
      // the module's visual colour so that the UI matches the original
      // Reactable patch (e.g. coloured loops, sample players, oscillators
      // with subtype-specific colours, etc.). When the attribute is not
      // present, fall back to a dark neutral default (#111111) so that
      // all modules tienen al menos un color visible y uniforme.
      std::uint32_t colour = module->colour_argb();
      const auto itColor = attrs.find("color");
      if (itColor != attrs.end() && !itColor->second.empty()) {
        colour = ParseColourArgb(attrs, "color", module->colour_argb());
      } else {
        colour = 0xFF111111U;
      }
      module->OverrideColour(colour);

      // For the Output (master) tangible we also mirror this colour into
      // the metadata used by the UI to tint the central node and its
      // pulses.
      if (metadata != nullptr && type == "Output") {
        metadata->master_colour_argb = colour;
      }

      (void)scene.AddModule(std::move(module));
    }

    // Create corresponding ObjectInstance. Reactable uses a `docked` flag to
    // indicate that certain tangibles live in a dock/toolbar area instead of
    // the main musical surface.
    const float x = ParseFloat(attrs, "x", 0.0F);
    const float y = ParseFloat(attrs, "y", 0.0F);
    const float angle_deg = ParseFloat(attrs, "angle", 0.0F);
    const float angle_rad = DegreesToRadians(angle_deg);
    const bool docked = ParseBool(attrs, "docked", false);

    ObjectInstance obj(static_cast<std::int64_t>(tangible_id), module_id,
               x, y, angle_rad, docked);
    scene.UpsertObject(obj);
  }

  // Derive rectai::Connection instances from hardlinks stored in DelayModule
  // instances. For each hardlink `to` id, we connect the delay's audio output
  // to the target module's main audio input. These connections are marked as
  // `is_hardlink = true` so that the Scene/UI can treat them as logically
  // active regardless of geometric connection constraints.
  for (const auto& [module_id, module_ptr] : scene.modules()) {
    if (module_ptr == nullptr) {
      continue;
    }

    const auto* delay = dynamic_cast<const DelayModule*>(module_ptr.get());
    if (delay == nullptr) {
      continue;
    }

    for (const int target_tangible_id : delay->hardlink_targets()) {
      const auto it = tangible_to_module_id.find(target_tangible_id);
      if (it == tangible_to_module_id.end()) {
        continue;
      }
      const std::string& target_module_id = it->second;

      Connection connection{.from_module_id = module_id,
                            .from_port_name = "out",
                            .to_module_id = target_module_id,
                            .to_port_name = "in",
                            .is_hardlink = true};
      (void)scene.AddConnection(connection);
    }
  }

  // Auto-connect Oscillator modules to the closest Filter module when
  // possible. This uses the ObjectInstance positions (normalized [0,1]
  // coordinates) to pick the nearest Filter tangible for each Oscillator.
  // The Scene model enforces at most one non-hardlink outgoing connection
  // per module, so each Oscillator will route dynamically to a single
  // Filter when available.
  {
    // Map module id -> ObjectInstance for distance computations.
    std::unordered_map<std::string, const ObjectInstance*> module_to_object;
    module_to_object.reserve(scene.objects().size());

    for (const auto& [tracking_id, obj] : scene.objects()) {
      (void)tracking_id;
      module_to_object.emplace(obj.logical_id(), &obj);
    }

    // Collect Filter module ids and keep a pointer to their AudioModule.
    struct FilterEntry {
      std::string id;
      const AudioModule* module{nullptr};
    };

    std::vector<FilterEntry> filters;
    filters.reserve(scene.modules().size());

    for (const auto& [module_id, module_ptr] : scene.modules()) {
      if (module_ptr == nullptr) {
        continue;
      }

      const auto* filter =
          dynamic_cast<const FilterModule*>(module_ptr.get());
      if (filter == nullptr) {
        continue;
      }

      filters.push_back(FilterEntry{module_id, filter});
    }

    if (!filters.empty()) {
      for (const auto& [module_id, module_ptr] : scene.modules()) {
        if (module_ptr == nullptr) {
          continue;
        }

        const auto* osc =
            dynamic_cast<const OscillatorModule*>(module_ptr.get());
        if (osc == nullptr) {
          continue;
        }

        const auto objIt = module_to_object.find(module_id);
        if (objIt == module_to_object.end() || objIt->second == nullptr) {
          continue;
        }

        const ObjectInstance* const oscObj = objIt->second;

        float bestDistSq = std::numeric_limits<float>::max();
        const FilterEntry* bestFilter = nullptr;

        for (const auto& entry : filters) {
          const auto objFilterIt =
              module_to_object.find(entry.id);
          if (objFilterIt == module_to_object.end() ||
              objFilterIt->second == nullptr) {
            continue;
          }

          const ObjectInstance* const filterObj = objFilterIt->second;
          const float dx = oscObj->x() - filterObj->x();
          const float dy = oscObj->y() - filterObj->y();
          const float distSq = dx * dx + dy * dy;

          if (distSq < bestDistSq && osc->CanConnectTo(*entry.module)) {
            bestDistSq = distSq;
            bestFilter = &entry;
          }
        }

        if (bestFilter != nullptr) {
          Connection connection{.from_module_id = module_id,
                                .from_port_name = "out",
                                .to_module_id = bestFilter->id,
                                .to_port_name = "in",
                                .is_hardlink = false};
          (void)scene.AddConnection(connection);
        }
      }
    }
  }

  // Auto-connect audio-capable modules to the Output (master) module when
  // present. Reactable patches use a tangible with id -1 to represent the
  // master output; in the Scene model this becomes an OutputModule with
  // module id -1 (MASTER_OUTPUT_ID) that should act as the common sink for chains. The
  // Output tangible itself remains invisible in the UI, but other modules are
  // allowed to connect to it logically via Scene::connections.
  const auto masterMappingIt = tangible_to_module_id.find(-1);
  if (masterMappingIt != tangible_to_module_id.end()) {
    const std::string& master_module_id = masterMappingIt->second;
    const AudioModule* const master_module = scene.FindModule(master_module_id);

    if (master_module != nullptr) {
      for (const auto& [module_id, module_ptr] : scene.modules()) {
        if (module_ptr == nullptr) {
          continue;
        }

        // Skip the master module itself and any global controllers such as
        // Volume, Tempo or Tonalizer, which must not participate in the
        // explicit connection graph.
        if (module_id == master_module_id ||
            module_ptr->is_global_controller()) {
          continue;
        }

        // Only create a link when the source can route signal to the
        // Output module according to the AudioModule graph rules (audio
        // compatibility, allowed target types, etc.). This also naturally
        // excludes MIDI-only modules such as the Sequencer.
        if (!module_ptr->CanConnectTo(*master_module)) {
          continue;
        }

        // Prefer the standard "out" -> "in" audio path; Scene::AddConnection
        // will reject duplicates (including existing hardlinks) and enforce
        // any additional validity checks.
        Connection connection{.from_module_id = module_id,
                              .from_port_name = "out",
                              .to_module_id = master_module_id,
                              .to_port_name = "in",
                              .is_hardlink = false};
        (void)scene.AddConnection(connection);
      }
    }
  }

  return true;
}

bool LoadReactablePatchFromFile(const std::string& path, Scene& scene,
                                ReactablePatchMetadata* const metadata,
                                std::string* const error_message)
{
  std::ifstream in(path);
  if (!in.is_open()) {
    if (error_message != nullptr) {
      *error_message = "Failed to open file: " + path;
    }
    return false;
  }

  std::ostringstream buffer;
  buffer << in.rdbuf();
  const std::string xml = buffer.str();
  return LoadReactablePatchFromString(xml, scene, metadata, error_message);
}

}  // namespace rectai
