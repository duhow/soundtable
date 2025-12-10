#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rectai {

// Represents a tangible or virtual object instance on the table.
class ObjectInstance {
 public:
  ObjectInstance() = default;
  ObjectInstance(std::int64_t tracking_id, std::string logical_id, float x,
                 float y, float angle_radians, bool docked = false);

  [[nodiscard]] std::int64_t tracking_id() const { return tracking_id_; }
  [[nodiscard]] const std::string& logical_id() const { return logical_id_; }
  [[nodiscard]] float x() const { return x_; }
  [[nodiscard]] float y() const { return y_; }
  [[nodiscard]] float angle_radians() const { return angle_radians_; }
  [[nodiscard]] bool docked() const { return docked_; }

  void set_position(float x, float y);
  void set_angle_radians(float angle_radians);

 private:
  std::int64_t tracking_id_{0};
  std::string logical_id_;
  float x_{0.0F};
  float y_{0.0F};
  float angle_radians_{0.0F};
  bool docked_{false};
};

// High-level category so UI/logic can reason about module families.
enum class ModuleType {
  kSequencer = 0,
  kAudio,
  kGenerator,
  kFilter,
  kSettings,
};

// Describes a module port (audio or control).
struct PortDescriptor {
  std::string name;
  bool is_audio{true};
};

// Base class for audio modules placed on the scene.
class AudioModule {
 public:
  AudioModule(std::string id, ModuleType type, bool produces_audio,
              bool consumes_audio);
  virtual ~AudioModule() = default;

  AudioModule(const AudioModule&) = delete;
  AudioModule& operator=(const AudioModule&) = delete;
  AudioModule(AudioModule&&) = delete;
  AudioModule& operator=(AudioModule&&) = delete;

  [[nodiscard]] const std::string& id() const { return id_; }
  [[nodiscard]] ModuleType type() const { return type_; }
  [[nodiscard]] bool produces_audio() const { return produces_audio_; }
  [[nodiscard]] bool consumes_audio() const { return consumes_audio_; }

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

  void AddInputPort(const std::string& name, bool is_audio);
  void AddOutputPort(const std::string& name, bool is_audio);

  void SetParameter(const std::string& name, float value);
  [[nodiscard]] float GetParameterOrDefault(const std::string& name,
                                            float default_value) const;

  // Per-parameter default value, so UI code does not need to hard-code
  // fallback values for things like "freq" or "gain". Concrete
  // AudioModule implementations can override this to provide
  // module-specific defaults.
  [[nodiscard]] virtual float default_parameter_value(
      const std::string& name) const;

  // Global controllers are special modules (e.g. Volume, Tempo,
  // Tonalizer) that influence session-wide state but are not part of
  // the audio routing graph. They must not participate in
  // Scene::connections (no dynamic links or hardlinks).
  [[nodiscard]] virtual bool is_global_controller() const { return false; }

  [[nodiscard]] bool CanConnectTo(const AudioModule& other) const;

 protected:
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
  bool allow_any_target_{true};
  std::unordered_set<ModuleType> allowed_targets_;
  std::vector<PortDescriptor> input_ports_;
  std::vector<PortDescriptor> output_ports_;
  std::unordered_map<std::string, float> parameters_;
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
};

// Complete scene: modules, connections and objects on the table.
class Scene {
 public:
  Scene() = default;

  // Gestión de módulos.
  bool AddModule(std::unique_ptr<AudioModule> module);
  bool RemoveModule(const std::string& module_id);

  // Gestión de conexiones.
  bool AddConnection(const Connection& connection);
  bool RemoveConnection(const std::string& from_module_id,
                        const std::string& from_port_name,
                        const std::string& to_module_id,
                        const std::string& to_port_name);

  // Gestión de objetos tangibles/virtuales.
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
