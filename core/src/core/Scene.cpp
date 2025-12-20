#include "core/Scene.h"

#include <algorithm>

namespace rectai {

// Canonical id used for the invisible master Output module.
inline constexpr const char* MASTER_OUTPUT_ID = "-1";

ObjectInstance::ObjectInstance(const std::int64_t tracking_id,
                               std::string logical_id, const float x,
                               const float y, const float angle_radians,
                               const bool docked)
    : tracking_id_(tracking_id),
      logical_id_(std::move(logical_id)),
      x_(x),
      y_(y),
      angle_radians_(angle_radians),
      docked_(docked),
      inside_music_area_(false) {}

ObjectInstance::ObjectInstance(const std::int64_t tracking_id,
                               std::string logical_id, const float x,
                               const float y, const float angle_radians,
                               const float velocity_x,
                               const float velocity_y,
                               const float angular_velocity,
                               const bool docked)
    : tracking_id_(tracking_id),
      logical_id_(std::move(logical_id)),
      x_(x),
      y_(y),
      angle_radians_(angle_radians),
      velocity_x_(velocity_x),
      velocity_y_(velocity_y),
      angular_velocity_(angular_velocity),
      docked_(docked),
      inside_music_area_(false) {}

void ObjectInstance::set_position(const float x, const float y)
{
  x_ = x;
  y_ = y;
}

void ObjectInstance::set_angle_radians(const float angle_radians)
{
  angle_radians_ = angle_radians;
}

void ObjectInstance::set_inside_music_area(const bool inside_music_area)
{
  inside_music_area_ = inside_music_area;
}

AudioModule::AudioModule(std::string id, const ModuleType type,
                         const bool produces_audio, const bool consumes_audio,
                         const bool produces_midi, const bool consumes_midi)
    : id_(std::move(id)),
      type_(type),
      produces_audio_(produces_audio),
      consumes_audio_(consumes_audio),
      produces_midi_(produces_midi),
      consumes_midi_(consumes_midi) {}

void AudioModule::AddInputPort(const std::string& name,
                               const PortSignalKind kind)
{
  input_ports_.push_back(PortDescriptor{name, kind});
}

void AudioModule::AddOutputPort(const std::string& name,
                                const PortSignalKind kind)
{
  output_ports_.push_back(PortDescriptor{name, kind});
}

void AudioModule::SetParameter(const std::string& name, const float value)
{
  parameters_[name] = value;
}

float AudioModule::GetParameterOrDefault(const std::string& name,
                                         const float default_value) const
{
  const auto it = parameters_.find(name);
  if (it == parameters_.end()) {
    return default_value;
  }
  return it->second;
}

float AudioModule::default_parameter_value(const std::string& /*name*/) const
{
  // Base implementation: no specific knowledge of parameters; concrete
  // modules (Oscillator, Filter, etc.) are expected to override this
  // to provide meaningful defaults for names like "freq" or "gain".
  return 0.0F;
}

std::optional<AudioModule::XYControlMapping>
AudioModule::xy_control_mapping() const
{
  // By default modules do not participate in the XYControl UI; audio
  // modules that want to expose a 2D pad in their detail panel can
  // override this to describe which parameters are driven by X and Y.
  return std::nullopt;
}

const AudioModuleModes& AudioModule::supported_modes() const
{
  static const AudioModuleModes kEmptyModes;
  return kEmptyModes;
}

int AudioModule::current_mode_index() const
{
  const auto& modes = supported_modes();
  if (modes.empty()) {
    // Modules without modes keep the legacy behaviour.
    return -1;
  }
  return current_mode_index_;
}

int AudioModule::default_mode_index() const
{
  const auto& modes = supported_modes();
  if (modes.empty()) {
    return -1;
  }
  // For modules that expose at least one mode, the default is the
  // first entry (index 0).
  return 0;
}

const AudioModuleMode* AudioModule::current_mode() const
{
  const auto& modes = supported_modes();
  const int index = current_mode_index();
  if (index < 0 || index >= static_cast<int>(modes.size())) {
    return nullptr;
  }
  return &modes[static_cast<std::size_t>(index)];
}

void AudioModule::set_mode(const int index)
{
  const auto& modes = supported_modes();
  const int count = static_cast<int>(modes.size());
  if (count <= 0) {
    return;
  }

  if (index < 0 || index >= count) {
    // Out-of-range indices are ignored so callers can safely pass
    // arbitrary values without disturbing the current mode.
    return;
  }

  if (index == current_mode_index_) {
    return;
  }

  current_mode_index_ = index;
  on_mode_changed(index, modes[static_cast<std::size_t>(index)]);
}

void AudioModule::set_mode(const std::string& subtype)
{
  const auto& modes = supported_modes();
  const auto it = std::find_if(
      modes.cbegin(), modes.cend(),
      [&subtype](const AudioModuleMode& mode) {
        return mode.type == subtype;
      });
  if (it == modes.cend()) {
    return;
  }

  const int index = static_cast<int>(
      std::distance(modes.cbegin(), it));
  set_mode(index);
}

void AudioModule::cycle_mode_forward()
{
  const auto& modes = supported_modes();
  const int count = static_cast<int>(modes.size());
  if (count <= 0) {
    return;
  }

  int current = current_mode_index();
  if (current < 0 || current >= count) {
    current = default_mode_index();
    if (current < 0 || current >= count) {
      current = 0;
    }
  }

  const int next = (current + 1) % count;
  set_mode(next);
}

void AudioModule::cycle_mode_backward()
{
  const auto& modes = supported_modes();
  const int count = static_cast<int>(modes.size());
  if (count <= 0) {
    return;
  }

  int current = current_mode_index();
  if (current < 0 || current >= count) {
    current = default_mode_index();
    if (current < 0 || current >= count) {
      current = 0;
    }
  }

  const int next = (current - 1 + count) % count;
  set_mode(next);
}

void AudioModule::on_mode_changed(const int /*newIndex*/, const AudioModuleMode& mode)
{
  // Default implementation: keep the visual icon in sync with the
  // selected mode metadata when an icon id is provided.
  if (!mode.icon_id.empty()) {
    set_icon_id(mode.icon_id);
  }
}

void AudioModule::set_connection_targets(
    std::unordered_set<ModuleType> targets)
{
  allowed_targets_ = std::move(targets);
  allow_any_target_ = allowed_targets_.empty();
}

void AudioModule::allow_any_connection_target()
{
  allow_any_target_ = true;
  allowed_targets_.clear();
}

bool AudioModule::CanConnectTo(const AudioModule& other) const
{
  if (is_global_controller() || other.is_global_controller()) {
    return false;
  }

  const bool audio_ok = produces_audio_ && other.consumes_audio_;
  const bool midi_ok = produces_midi_ && other.consumes_midi_;
  if (!audio_ok && !midi_ok) {
    return false;
  }

  if (allow_any_target_) {
    return true;
  }

  return allowed_targets_.find(other.type_) != allowed_targets_.end();
}

const AudioModule::SettingsTabs& AudioModule::supported_settings_tabs()
    const
{
  // By default, expose only the generic Settings tab using the
  // standard icon from the atlas. Concrete modules can override this
  // to add Envelope or module-specific tabs.
  static const SettingsTabs kTabs = {
      SettingsTabDescriptor{SettingsTabKind::kSettings, "tab_settings"},
  };
  return kTabs;
}

bool Scene::AddModule(std::unique_ptr<AudioModule> module)
{
  if (!module) {
    return false;
  }

  const auto id = module->id();
  if (modules_.find(id) != modules_.end()) {
    return false;
  }

  modules_.emplace(id, std::move(module));
  return true;
}

bool Scene::RemoveModule(const std::string& module_id)
{
  const auto erased = modules_.erase(module_id);
  if (erased == 0U) {
    return false;
  }

  // Remove any connections that involve the deleted module.
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
  auto* fromModule = FindModule(connection.from_module_id);
  auto* toModule = FindModule(connection.to_module_id);
  if (fromModule == nullptr || toModule == nullptr) {
    return false;
  }

  // Global controller modules (Volume, Tempo, Tonalizer, etc.) must
  // never participate in the explicit connection graph: they affect
  // session-level state only.
  if (fromModule->is_global_controller() ||
      toModule->is_global_controller()) {
    return false;
  }

  if (!fromModule->CanConnectTo(*toModule)) {
    return false;
  }

  // Enforce that a module can have at most one non-hardlink (dynamic)
  // outgoing connection **to other modules**, while still allowing an
  // additional non-hardlink connection to the master Output
  // (MASTER_OUTPUT_ID). This
  // keeps the "single dynamic route" invariant for spatial routing
  // (e.g. Osc → closest Filter) without blocking the implicit
  // module→Output(-1) auto-wiring used to model radial lines and
  // connection-level mute to the master.
    if (!connection.is_hardlink &&
      connection.to_module_id != rectai::MASTER_OUTPUT_ID) {
    const auto existingOut = std::find_if(
        connections_.cbegin(), connections_.cend(),
        [&connection](const Connection& c) {
          return c.from_module_id == connection.from_module_id &&
                 c.to_module_id != rectai::MASTER_OUTPUT_ID;
        });
    if (existingOut != connections_.cend()) {
      return false;
    }
  }

  // Ensure there is no identical connection already present.
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

bool Scene::RemoveConnection(const Connection& connection)
{
  return RemoveConnection(connection.from_module_id, connection.from_port_name,
                          connection.to_module_id, connection.to_port_name);
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
  // Ensure that there is at most one ObjectInstance per logical_id in
  // the scene. When a new object is upserted (either from a loaded
  // patch or from live tracking via OSC), remove any existing objects
  // that reference the same logical_id but use a different
  // tracking_id. This guarantees that the most recent source
  // (typically OSC) wins over any previous manual position.
  for (auto it = objects_.begin(); it != objects_.end();) {
    if (it->second.logical_id() == object.logical_id() &&
        it->first != object.tracking_id()) {
      it = objects_.erase(it);
    } else {
      ++it;
    }
  }

  objects_[object.tracking_id()] = object;
}

void Scene::RemoveObject(const std::int64_t tracking_id)
{
  (void)objects_.erase(tracking_id);
}

void Scene::SetModuleParameter(const std::string& module_id,
                               const std::string& name, const float value)
{
  const auto it = modules_.find(module_id);
  if (it == modules_.end()) {
    return;
  }
  it->second->SetParameter(name, value);
}

float Scene::GetModuleParameterOrDefault(const std::string& module_id,
                                         const std::string& name,
                                         const float default_value) const
{
  const auto it = modules_.find(module_id);
  if (it == modules_.end()) {
    return default_value;
  }
  return it->second->GetParameterOrDefault(name, default_value);
}

AudioModule* Scene::FindModule(const std::string& module_id)
{
  const auto it = modules_.find(module_id);
  if (it == modules_.end()) {
    return nullptr;
  }
  return it->second.get();
}

const AudioModule* Scene::FindModule(const std::string& module_id) const
{
  const auto it = modules_.find(module_id);
  if (it == modules_.end()) {
    return nullptr;
  }
  return it->second.get();
}

}  // namespace rectai
