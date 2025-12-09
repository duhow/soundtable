#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace rectai {

// Representa una instancia de objeto tangible o virtual sobre la mesa.
class ObjectInstance {
 public:
  ObjectInstance() = default;
  ObjectInstance(std::int64_t tracking_id, std::string logical_id, float x,
                 float y, float angle_radians);

  [[nodiscard]] std::int64_t tracking_id() const { return tracking_id_; }
  [[nodiscard]] const std::string& logical_id() const { return logical_id_; }
  [[nodiscard]] float x() const { return x_; }
  [[nodiscard]] float y() const { return y_; }
  [[nodiscard]] float angle_radians() const { return angle_radians_; }

  void set_position(float x, float y);
  void set_angle_radians(float angle_radians);

 private:
  std::int64_t tracking_id_{0};
  std::string logical_id_;
  float x_{0.0F};
  float y_{0.0F};
  float angle_radians_{0.0F};
};

// Tipo de módulo lógico (sintetizador, filtro, etc.).
enum class ModuleKind {
  kUnknown = 0,
  kOscillator,
  kFilter,
  kEffect,
  kSampler,
  kController,
};

// Representa un puerto de un módulo (audio o control).
struct PortDescriptor {
  std::string name;
  bool is_audio{true};
};

// Módulo lógico dentro de la escena.
class Module {
 public:
  Module() = default;
  Module(std::string id, ModuleKind kind);

  [[nodiscard]] const std::string& id() const { return id_; }
  [[nodiscard]] ModuleKind kind() const { return kind_; }

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

 private:
  std::string id_;
  ModuleKind kind_{ModuleKind::kUnknown};
  std::vector<PortDescriptor> input_ports_;
  std::vector<PortDescriptor> output_ports_;
  std::unordered_map<std::string, float> parameters_;
};

// Conexión dirigida entre puertos de módulos.
struct Connection {
  std::string from_module_id;
  std::string from_port_name;
  std::string to_module_id;
  std::string to_port_name;
};

// Escena completa: módulos, conexiones y objetos sobre la mesa.
class Scene {
 public:
  Scene() = default;

  // Gestión de módulos.
  bool AddModule(const Module& module);
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

  [[nodiscard]] const std::unordered_map<std::string, Module>& modules() const {
    return modules_;
  }

  [[nodiscard]] const std::vector<Connection>& connections() const {
    return connections_;
  }

  [[nodiscard]] const std::unordered_map<std::int64_t, ObjectInstance>&
  objects() const {
    return objects_;
  }

 private:
  std::unordered_map<std::string, Module> modules_;
  std::vector<Connection> connections_;
  std::unordered_map<std::int64_t, ObjectInstance> objects_;
};

}  // namespace rectai
