#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <typeinfo>

namespace rectai {

// Represents a tangible or virtual object instance on the table.
class ObjectInstance {
 public:
  ObjectInstance() = default;
  ObjectInstance(std::int64_t tracking_id, std::string logical_id, float x,
                 float y, float angle_radians, bool docked = false);

  ObjectInstance(std::int64_t tracking_id, std::string logical_id, float x,
                 float y, float angle_radians, float velocity_x,
                 float velocity_y, float angular_velocity,
                 bool docked = false);

  [[nodiscard]] std::int64_t tracking_id() const { return tracking_id_; }
  [[nodiscard]] const std::string& logical_id() const { return logical_id_; }
  [[nodiscard]] float x() const { return x_; }
  [[nodiscard]] float y() const { return y_; }
  [[nodiscard]] float angle_radians() const { return angle_radians_; }
  [[nodiscard]] bool docked() const { return docked_; }

  [[nodiscard]] float velocity_x() const { return velocity_x_; }
  [[nodiscard]] float velocity_y() const { return velocity_y_; }
  [[nodiscard]] float angular_velocity() const { return angular_velocity_; }

  [[nodiscard]] bool inside_music_area() const
  {
    return inside_music_area_;
  }

  void set_position(float x, float y);
  void set_angle_radians(float angle_radians);
  void set_inside_music_area(bool inside_music_area);

 private:
  std::int64_t tracking_id_{0};
  std::string logical_id_;
  float x_{0.0F};
  float y_{0.0F};
  float angle_radians_{0.0F};
  float velocity_x_{0.0F};
  float velocity_y_{0.0F};
  float angular_velocity_{0.0F};
  bool docked_{false};
  bool inside_music_area_{false};
};

// High-level category so UI/logic can reason about module families.
enum class ModuleType {
  kSequencer = 0,
  kAudio,
  kGenerator,
  kFilter,
  kSettings,
};

// Kind of signal transported by a port.
enum class PortSignalKind {
  kAudio = 0,
  kMidi,
  kControl,
};

// Describes a module port (audio, MIDI or control).
struct PortDescriptor {
  std::string name;
  PortSignalKind kind{PortSignalKind::kAudio};
};

// Lightweight description of per-module adjustment modes that can be
// exposed to the UI (for example, filter type or oscillator waveform).
// Each mode carries a stable numeric id, a short type string (e.g.
// "sine", "lowpass") and the icon identifier used by the atlas.
struct AudioModuleMode {
  int id{0};
  std::string type;
  std::string icon_id;
};

using AudioModuleModes = std::vector<AudioModuleMode>;

// Base class for audio modules placed on the scene.
class AudioModule {
 public:
  AudioModule(std::string id, ModuleType type, bool produces_audio,
              bool consumes_audio, bool produces_midi = false,
              bool consumes_midi = false);
  virtual ~AudioModule() = default;

  AudioModule(const AudioModule&) = delete;
  AudioModule& operator=(const AudioModule&) = delete;
  AudioModule(AudioModule&&) = delete;
  AudioModule& operator=(AudioModule&&) = delete;

  [[nodiscard]] const std::string& id() const { return id_; }
  [[nodiscard]] ModuleType type() const { return type_; }
  [[nodiscard]] bool produces_audio() const { return produces_audio_; }
  [[nodiscard]] bool consumes_audio() const { return consumes_audio_; }
  [[nodiscard]] bool produces_midi() const { return produces_midi_; }
  [[nodiscard]] bool consumes_midi() const { return consumes_midi_; }

  [[nodiscard]] bool uses_frequency_control() const {
    return uses_frequency_control_;
  }

  [[nodiscard]] bool uses_gain_control() const { return uses_gain_control_; }

  // Simple synthesis defaults for UI-driven engines.
  [[nodiscard]] double base_frequency_hz() const { return base_frequency_hz_; }
  [[nodiscard]] double frequency_range_hz() const
  {
    return frequency_range_hz_;
  }
  [[nodiscard]] float base_level() const { return base_level_; }
  [[nodiscard]] float level_range() const { return level_range_; }

  [[nodiscard]] uint32_t colour_argb() const { return colour_argb_; }
  [[nodiscard]] const std::string& label() const { return label_; }
  [[nodiscard]] const std::string& description() const { return description_; }
  [[nodiscard]] const std::string& icon_id() const { return icon_id_; }

  [[nodiscard]] const std::vector<PortDescriptor>& input_ports() const {
    return input_ports_;
  }

  [[nodiscard]] const std::vector<PortDescriptor>& output_ports() const {
    return output_ports_;
  }

  void AddInputPort(const std::string& name, PortSignalKind kind);
  void AddOutputPort(const std::string& name, PortSignalKind kind);

  void SetParameter(const std::string& name, float value);
  [[nodiscard]] float GetParameterOrDefault(const std::string& name,
                                            float default_value) const;

  // Per-parameter default value, so UI code does not need to hard-code
  // fallback values for things like "freq" or "gain". Concrete
  // AudioModule implementations can override this to provide
  // module-specific defaults.
  [[nodiscard]] virtual float default_parameter_value(
      const std::string& name) const;

  // Optional per-module set of adjustment modes (for example,
  // Oscillator waveforms or Filter types) that can be cycled from the
  // UI. By default modules do not expose any modes; concrete
  // implementations can override these methods to participate in the
  // generic mode-selection UI.
  [[nodiscard]] virtual const AudioModuleModes& supported_modes() const;

  // Returns the zero-based index of the currently active mode in
  // supported_modes(), or -1 when the module does not expose
  // selectable modes.
  [[nodiscard]] virtual int current_mode_index() const;

  // Convenience accessor that returns a pointer to the currently
  // active AudioModuleMode, or nullptr when the module does not
  // expose selectable modes or the active index is out of range.
  [[nodiscard]] const AudioModuleMode* current_mode() const;

  // Default mode index for modules that expose modes. Implementations
  // that participate in the mode API should ensure that
  // supported_modes() is non-empty and that default_mode_index()
  // returns a valid index within that vector.
  [[nodiscard]] virtual int default_mode_index() const;

  // Update the current mode by index. The default implementation
  // validates the index against supported_modes(), keeps the current
  // mode unchanged when the index is out of range and, on success,
  // updates the internal index and calls on_mode_changed().
  virtual void set_mode(int index);
  virtual void set_mode(const std::string& subtype);

  // Convenience helpers to cycle through supported modes in a generic
  // way (forward / backward). Modules without modes simply ignore
  // these calls.
  void cycle_mode_forward();
  void cycle_mode_backward();

  // Global controllers are special modules (e.g. Volume, Tempo,
  // Tonalizer) that influence session-wide state but are not part of
  // the audio routing graph. They must not participate in
  // Scene::connections (no dynamic links or hardlinks).
  [[nodiscard]] virtual bool is_global_controller() const { return false; }

  [[nodiscard]] bool CanConnectTo(const AudioModule& other) const;

  // Check if the module matches the same concrete C++ type (subclass).
  template<typename T>
  [[nodiscard]] bool is() const {
      return dynamic_cast<const T*>(this) != nullptr;
  }

  // Allows loaders or UI code to override the visual colour of a
  // module based on external data (e.g. colours stored in a Reactable
  // .rtp patch). Concrete modules still configure a sensible default
  // in their constructors; this is an optional override on top.
  void OverrideColour(const uint32_t argb) { set_colour(argb); }

 protected:
  // Hook invoked whenever set_mode updates the active
  // mode. The default implementation keeps the visual icon in sync
  // with the selected mode (using AudioModuleMode::icon_id); concrete
  // modules that participate in the mode system can override this to
  // map the new index (and associated AudioModuleMode) onto their
  // internal representation (e.g. waveform, filter type) and may
  // optionally call the base implementation to reuse the icon
  // behaviour.
  virtual void on_mode_changed(int newIndex, const AudioModuleMode& mode);

  void set_colour(uint32_t argb) { colour_argb_ = argb; }
  void set_label(std::string label) { label_ = std::move(label); }
  void set_description(std::string description)
  {
    description_ = std::move(description);
  }
  void set_icon_id(std::string icon_id) { icon_id_ = std::move(icon_id); }
  void enable_frequency_control(bool enabled)
  {
    uses_frequency_control_ = enabled;
  }
  void enable_gain_control(bool enabled) { uses_gain_control_ = enabled; }
  void set_connection_targets(std::unordered_set<ModuleType> targets);
  void allow_any_connection_target();

  void set_frequency_mapping(double base_hz, double range_hz)
  {
    base_frequency_hz_ = base_hz;
    frequency_range_hz_ = range_hz;
  }

  void set_level_mapping(float base, float range)
  {
    base_level_ = base;
    level_range_ = range;
  }

 private:
  std::string id_;
  ModuleType type_;
  bool produces_audio_{false};
  bool consumes_audio_{false};
  bool uses_frequency_control_{false};
  bool uses_gain_control_{false};
  double base_frequency_hz_{200.0};
  double frequency_range_hz_{800.0};
  float base_level_{0.02F};
  float level_range_{0.18F};
  uint32_t colour_argb_{0xFF2090FF};
  std::string label_;
  std::string description_;
  std::string icon_id_;
  bool produces_midi_{false};
  bool consumes_midi_{false};
  bool allow_any_target_{true};
  std::unordered_set<ModuleType> allowed_targets_;
  std::vector<PortDescriptor> input_ports_;
  std::vector<PortDescriptor> output_ports_;
  std::unordered_map<std::string, float> parameters_;
  // -1 means "no mode selected yet" so that the first call to
  // set_mode(index) will always trigger on_mode_changed even when
  // index is 0. Modules without modes keep returning -1 via
  // current_mode_index().
  int current_mode_index_{-1};
};

// Directed connection between module ports. A connection can optionally
// be marked as a hardlink, meaning that from the perspective of the
// scene and UI it should remain logically active even if the usual
// geometric constraints for dynamic connections (e.g. angular cone)
// are not satisfied.
struct Connection {
  std::string from_module_id;
  std::string from_port_name;
  std::string to_module_id;
  std::string to_port_name;
  bool is_hardlink{false};
  // Optional per-connection mute flag. This captures the persistent
  // mute state loaded from sessions (e.g. Reactable hardlinks with
  // muted="1"). Runtime UI gestures may overlay additional mute
  // state on top of this via higher-level structures.
  bool muted{false};

  [[nodiscard]] bool operator==(const Connection& other) const {
    return from_module_id == other.from_module_id &&
           from_port_name == other.from_port_name &&
           to_module_id == other.to_module_id &&
           to_port_name == other.to_port_name;
  }

  [[nodiscard]] bool operator!=(const Connection& other) const {
    return !(*this == other);
  }
};

// Complete scene: modules, connections and objects on the table.
class Scene {
 public:
  Scene() = default;

  // Module management.
  bool AddModule(std::unique_ptr<AudioModule> module);
  bool RemoveModule(const std::string& module_id);

  // Connection management.
  bool AddConnection(const Connection& connection);
  bool RemoveConnection(const Connection& connection);
  bool RemoveConnection(const std::string& from_module_id,
                        const std::string& from_port_name,
                        const std::string& to_module_id,
                        const std::string& to_port_name);

  // Tangible/virtual object management.
  void UpsertObject(const ObjectInstance& object);
  void RemoveObject(std::int64_t tracking_id);

  using ModuleStore =
      std::unordered_map<std::string, std::unique_ptr<AudioModule>>;

  [[nodiscard]] const ModuleStore& modules() const { return modules_; }

  [[nodiscard]] AudioModule* FindModule(const std::string& module_id);
  [[nodiscard]] const AudioModule* FindModule(
      const std::string& module_id) const;

  void SetModuleParameter(const std::string& module_id,
                          const std::string& name, float value);

  [[nodiscard]] float GetModuleParameterOrDefault(
      const std::string& module_id, const std::string& name,
      float default_value) const;

  [[nodiscard]] const std::vector<Connection>& connections() const {
    return connections_;
  }

  [[nodiscard]] const std::unordered_map<std::int64_t, ObjectInstance>&
  objects() const {
    return objects_;
  }

 private:
  ModuleStore modules_;
  std::vector<Connection> connections_;
  std::unordered_map<std::int64_t, ObjectInstance> objects_;
};

}  // namespace rectai
