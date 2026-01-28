#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "core/Scene.h"

namespace soundtable {

// Logical audio graph derived from a Scene: nodes are AudioModules and
// edges are Scene::Connection entries annotated with their signal kind.
//
// Phase 2 of the "buffers per connection" plan: this graph does not
// run DSP yet, but serves as a stable representation of audio routing
// on top of which the engine will be able to attach per-connection
// taps in later phases.
class AudioGraph {
public:
    struct Node {
        std::string id;
        ModuleType type{ModuleType::kAudio};
        bool produces_audio{false};
        bool consumes_audio{false};
        bool produces_midi{false};
        bool consumes_midi{false};
        bool is_global_controller{false};
    };

    struct Edge {
        std::string from_module_id;
        std::string from_port_name;
        std::string to_module_id;
        std::string to_port_name;
        bool is_hardlink{false};
        bool muted{false};
        PortSignalKind signal_kind{PortSignalKind::kAudio};
    };

    AudioGraph() = default;

    // Rebuilds the complete graph from a current Scene snapshot. The
    // caller is responsible for any synchronization with the audio
    // thread (for now this is only used for logic and visualisation,
    // not for real-time processing).
    void RebuildFromScene(const Scene& scene);

    [[nodiscard]] const std::unordered_map<std::string, Node>& nodes() const noexcept
    {
        return nodes_;
    }

    [[nodiscard]] const std::vector<Edge>& edges() const noexcept
    {
        return edges_;
    }

    // Convenience: returns only edges that carry audio.
    [[nodiscard]] std::vector<Edge> audio_edges() const;

private:
    std::unordered_map<std::string, Node> nodes_;
    std::vector<Edge> edges_;
};

}  // namespace soundtable
