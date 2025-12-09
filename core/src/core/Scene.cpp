#include "core/Scene.h"

namespace rectai {

ObjectInstance::ObjectInstance(const std::int64_t tracking_id,
                               std::string logical_id, const float x,
                               const float y, const float angle_radians)
    : tracking_id_(tracking_id),
      logical_id_(std::move(logical_id)),
      x_(x),
      y_(y),
      angle_radians_(angle_radians) {}

void ObjectInstance::set_position(const float x, const float y)
{
  x_ = x;
  y_ = y;
}

void ObjectInstance::set_angle_radians(const float angle_radians)
{
  angle_radians_ = angle_radians;
}

Module::Module(std::string id, const ModuleKind kind)
    : id_(std::move(id)), kind_(kind) {}

void Module::AddInputPort(const std::string& name, const bool is_audio)
{
  input_ports_.push_back(PortDescriptor{name, is_audio});
}

void Module::AddOutputPort(const std::string& name, const bool is_audio)
{
  output_ports_.push_back(PortDescriptor{name, is_audio});
}

void Module::SetParameter(const std::string& name, const float value)
{
  parameters_[name] = value;
}

float Module::GetParameterOrDefault(const std::string& name,
                                    const float default_value) const
{
  const auto it = parameters_.find(name);
  if (it == parameters_.end()) {
    return default_value;
  }
  return it->second;
}

bool Scene::AddModule(const Module& module)
{
  const auto id = module.id();
  const auto [it, inserted] = modules_.emplace(id, module);
  (void)it;  // Silenciar warning en builds sin uso.
  return inserted;
}

bool Scene::RemoveModule(const std::string& module_id)
{
  const auto erased = modules_.erase(module_id);
  if (erased == 0U) {
    return false;
  }

  // Eliminar conexiones que involucren al módulo borrado.
  connections_.erase(
      std::remove_if(connections_.begin(), connections_.end(),
                     [&module_id](const Connection& c) {
                       return c.from_module_id == module_id ||
                              c.to_module_id == module_id;
                     }),
      connections_.end());

  return true;
}

bool Scene::AddConnection(const Connection& connection)
{
  // Comprobar que los módulos existen.
  if (modules_.find(connection.from_module_id) == modules_.end() ||
      modules_.find(connection.to_module_id) == modules_.end()) {
    return false;
  }

  // Comprobar que no exista ya una conexión idéntica.
  const auto it = std::find_if(
      connections_.cbegin(), connections_.cend(),
      [&connection](const Connection& c) {
        return c.from_module_id == connection.from_module_id &&
               c.from_port_name == connection.from_port_name &&
               c.to_module_id == connection.to_module_id &&
               c.to_port_name == connection.to_port_name;
      });
  if (it != connections_.cend()) {
    return false;
  }

  connections_.push_back(connection);
  return true;
}

bool Scene::RemoveConnection(const std::string& from_module_id,
                             const std::string& from_port_name,
                             const std::string& to_module_id,
                             const std::string& to_port_name)
{
  const auto original_size = connections_.size();

  connections_.erase(
      std::remove_if(connections_.begin(), connections_.end(),
                     [&from_module_id, &from_port_name, &to_module_id,
                      &to_port_name](const Connection& c) {
                       return c.from_module_id == from_module_id &&
                              c.from_port_name == from_port_name &&
                              c.to_module_id == to_module_id &&
                              c.to_port_name == to_port_name;
                     }),
      connections_.end());

  return connections_.size() != original_size;
}

void Scene::UpsertObject(const ObjectInstance& object)
{
  objects_[object.tracking_id()] = object;
}

void Scene::RemoveObject(const std::int64_t tracking_id)
{
  (void)objects_.erase(tracking_id);
}

}  // namespace rectai
