#include "core/SceneSerialization.h"

#include <sstream>

namespace rectai {

namespace {

int ToUnderlying(const ModuleKind kind)
{
    return static_cast<int>(kind);
}

}  // namespace

std::string SerializeScene(const Scene& scene)
{
    std::ostringstream out;

    out << "rectai_scene_v1" << '\n';

    // Modules.
    for (const auto& [id, module] : scene.modules()) {
        out << "module " << id << ' ' << ToUnderlying(module.kind()) << '\n';
    }

    // Objects.
    for (const auto& [tracking_id, object] : scene.objects()) {
        out << "object " << tracking_id << ' ' << object.logical_id() << ' '
            << object.x() << ' ' << object.y() << ' '
            << object.angle_radians() << '\n';
    }

    // Connections.
    for (const auto& connection : scene.connections()) {
        out << "connection " << connection.from_module_id << ' '
            << connection.from_port_name << ' ' << connection.to_module_id
            << ' ' << connection.to_port_name << '\n';
    }

    return out.str();
}

}  // namespace rectai
