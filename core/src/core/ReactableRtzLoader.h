// ReactableRtzLoader: utilities to import .rtz session archives.

#pragma once

#include <string>

#include "core/Scene.h"

namespace soundtable {

struct ReactablePatchMetadata;

// Import a Reactable .rtz archive into the local Reactable content tree and
// load the contained .rtp session into the provided Scene.
//
// The .rtz file is a ZIP archive that must contain at least one .rtp file.
// A single root-level .rtp is preferred when present, but if there is
// exactly one .rtp anywhere in the archive it will be used as the session
// file even when stored inside a subfolder. Optionally, the archive may
// also contain one or more folders whose last path component matches the
// .rtp basename (without extension) with audio samples (typically WAV
// files).
//
// Behaviour:
//  - Validates that `rtz_path` points to a ZIP file (checks PK header
//    and uses libzip for parsing).
//  - Selects the .rtp session file as follows:
//      * If there is exactly one root-level .rtp, it is used.
//      * Otherwise, if there is exactly one .rtp anywhere in the
//        archive, it is used.
//      * Archives with zero or multiple candidate .rtp files are
//        rejected.
//    The chosen .rtp is extracted into
//      <com_reactable_root>/Sessions/<rtp_entry_name>
//    preserving any subfolder structure from the archive.
//  - For each entry under a folder whose last path component matches
//    the .rtp basename (e.g. Loopdemo/ for Loopdemo.rtp), all files
//    are extracted into
//      <com_reactable_root>/Samples/<relative_path_inside_that_folder>
//    so that loop filenames from the .rtp (e.g.
//    "Demoloops/pl_padloop1.wav") map directly to
//    "Samples/Demoloops/pl_padloop1.wav". Hidden/temporary files
//    (whose last path component starts with '.') are skipped.
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

}  // namespace soundtable
