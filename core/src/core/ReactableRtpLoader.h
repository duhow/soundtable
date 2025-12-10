#pragma once

#include <string>

#include "core/Scene.h"

namespace rectai {

// Minimal metadata extracted from a Reactable .rtp file.
struct ReactablePatchMetadata {
  std::string author_name;
  std::string patch_name;
};

// Loads a Reactable .rtp XML document from a string and populates a Scene.
//
// The existing content of `scene` is cleared by the caller if needed; this
// function only adds modules, connections and objects based on the file.
//
// On failure, returns false and optionally fills `error_message`.
[[nodiscard]] bool LoadReactablePatchFromString(const std::string& xml,
                                                Scene& scene,
                                                ReactablePatchMetadata* metadata,
                                                std::string* error_message);

// Convenience helper to load directly from a file path.
[[nodiscard]] bool LoadReactablePatchFromFile(const std::string& path,
                                              Scene& scene,
                                              ReactablePatchMetadata* metadata,
                                              std::string* error_message);

}  // namespace rectai
