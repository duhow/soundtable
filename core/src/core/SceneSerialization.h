#pragma once

#include <string>

#include "core/Scene.h"

namespace rectai {

// Very simple, line-based textual serialization for rectai::Scene.
//
// Format (version 1):
//   rectai_scene_v1
//   module <id> <kind_int>
//   object <tracking_id> <logical_id> <x> <y> <angle_radians>
//   connection <from_module_id> <from_port_name> <to_module_id> <to_port_name>
//
// This is primarily intended for debugging and quick presets. It can be
// evolved or replaced by a JSON-based format later while keeping this as a
// minimal, human-readable option.

[[nodiscard]] std::string SerializeScene(const Scene& scene);

}  // namespace rectai
