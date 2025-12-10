#include "core/SceneSerialization.h"

#include <sstream>

namespace rectai {

namespace {

int ToUnderlying(const ModuleType type)
{
    return static_cast<int>(type);
}

}  // namespace

std::string SerializeScene(const Scene& scene)
{
    std::ostringstream out;

    out << "rectai_scene_v1" << '\n';

    // Modules.
    for (const auto& [id, module] : scene.modules()) {
        if (module == nullptr) {
            continue;
        }
        out << "module " << id << ' ' << ToUnderlying(module->type())
            << '\n';
    }

    // Objects.
    for (const auto& [tracking_id, object] : scene.objects()) {
        out << "object " << tracking_id << ' ' << object.logical_id() << ' '
            << object.x() << ' ' << object.y() << ' '
            << object.angle_radians() << '\n';
    }

    // Connections. Hardlink connections are annotated explicitly so that
    // tooling or future loaders can distinguish them from dynamic,
    // geometry-based connections when inspecting a serialized scene.
    for (const auto& connection : scene.connections()) {
        out << "connection " << connection.from_module_id << ' '
            << connection.from_port_name << ' ' << connection.to_module_id
            << ' ' << connection.to_port_name;
        if (connection.is_hardlink) {
            out << ' ' << "hardlink";
        }
        out << '\n';
    }

    return out.str();
}

}  // namespace rectai
