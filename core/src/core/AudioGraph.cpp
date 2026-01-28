#include "core/AudioGraph.h"

namespace soundtable {

namespace {

PortSignalKind InferSignalKind(const Scene::ModuleStore& modules,
                               const Connection& conn)
{
    const auto fromIt = modules.find(conn.from_module_id);
    if (fromIt != modules.end() && fromIt->second != nullptr) {
        const auto& fromModule = *fromIt->second;
        for (const auto& port : fromModule.output_ports()) {
            if (port.name == conn.from_port_name) {
                return port.kind;
            }
        }
        if (fromModule.produces_audio()) {
            return PortSignalKind::kAudio;
        }
    }

    const auto toIt = modules.find(conn.to_module_id);
    if (toIt != modules.end() && toIt->second != nullptr) {
        const auto& toModule = *toIt->second;
        for (const auto& port : toModule.input_ports()) {
            if (port.name == conn.to_port_name) {
                return port.kind;
            }
        }
        if (toModule.consumes_audio()) {
            return PortSignalKind::kAudio;
        }
    }

    // Conservative default: treat the connection as audio when the
    // signal kind cannot be inferred from module ports.
    return PortSignalKind::kAudio;
}

}  // namespace

void AudioGraph::RebuildFromScene(const Scene& scene)
{
    nodes_.clear();
    edges_.clear();

    const auto& modules = scene.modules();

    for (const auto& [id, modulePtr] : modules) {
        if (modulePtr == nullptr) {
            continue;
        }

        Node node;
        node.id = id;
        node.type = modulePtr->type();
        node.produces_audio = modulePtr->produces_audio();
        node.consumes_audio = modulePtr->consumes_audio();
        node.produces_midi = modulePtr->produces_midi();
        node.consumes_midi = modulePtr->consumes_midi();
        node.is_global_controller = modulePtr->is_global_controller();

        nodes_.emplace(id, std::move(node));
    }

    for (const auto& conn : scene.connections()) {
        Edge edge;
        edge.from_module_id = conn.from_module_id;
        edge.from_port_name = conn.from_port_name;
        edge.to_module_id = conn.to_module_id;
        edge.to_port_name = conn.to_port_name;
        edge.is_hardlink = conn.is_hardlink;
        edge.muted = conn.muted;
        edge.signal_kind = InferSignalKind(modules, conn);

        edges_.push_back(std::move(edge));
    }
}

std::vector<AudioGraph::Edge> AudioGraph::audio_edges() const
{
    std::vector<Edge> result;
    result.reserve(edges_.size());

    for (const auto& e : edges_) {
        if (e.signal_kind == PortSignalKind::kAudio) {
            result.push_back(e);
        }
    }

    return result;
}

}  // namespace soundtable
