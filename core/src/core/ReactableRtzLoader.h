// ReactableRtzLoader: utilities to import .rtz session archives.

#pragma once

#include <string>

#include "core/Scene.h"

namespace rectai {

struct ReactablePatchMetadata;

// Import a Reactable .rtz archive into the local com.reactable tree and
// load the contained .rtp session into the provided Scene.
//
// The .rtz file is a ZIP archive that must contain at least one .rtp file
// at the root of the archive. Optionally, it may also contain a folder
// whose name matches the .rtp basename (without extension) with audio
// samples (typically WAV files).
//
// Behaviour:
//  - Validates that `rtz_path` points to a ZIP file (checks PK header
//    and uses libzip for parsing).
//  - Ensures that the root of the archive contains exactly one .rtp
//    file; this file is extracted into
//      <com_reactable_root>/Sessions/<rtp_filename>
//  - If the archive contains a folder whose name matches the .rtp
//    basename (e.g. Loopdemo/ for Loopdemo.rtp), all files under that
//    folder are extracted into
//      <com_reactable_root>/Samples/<folder_name>/...
//    Hidden/temporary files (whose last path component starts with '.')
//    are skipped.
//  - When extracting, if a destination file already exists and is
//    byte‑for‑byte identical to the content in the archive, it is left
//    untouched. If it exists but differs, it is overwritten with the
//    version from the archive.
//  - After extraction, the resulting .rtp is loaded via
//    LoadReactablePatchFromFile to populate `scene` and `metadata`.
//
// On success, returns true and leaves `scene` populated. On failure,
// returns false and, when provided, fills `error_message` with a brief
// description of the problem.
[[nodiscard]] bool LoadReactableSessionFromRtz(
    const std::string& rtz_path,
    const std::string& com_reactable_root,
    Scene& scene,
    ReactablePatchMetadata* metadata,
    std::string* error_message);

}  // namespace rectai
