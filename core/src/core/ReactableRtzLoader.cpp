#include "core/ReactableRtzLoader.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#if defined(SOUNDTABLE_HAVE_LIBZIP)
#include <zip.h>
#endif

#include "core/ReactableRtpLoader.h"

namespace soundtable {
namespace {

namespace fs = std::filesystem;

std::string ToLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool HasZipSignature(const fs::path& path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    return false;
  }

  unsigned char header[4] = {0, 0, 0, 0};
  in.read(reinterpret_cast<char*>(header), 4);
  if (in.gcount() < 2) {
    return false;
  }

  // ZIP local/file headers start with "PK". We accept at least that
  // marker and, when available, the classic 0x03 0x04 signature.
  if (header[0] != 0x50U || header[1] != 0x4bU) {  // 'P''K'
    return false;
  }

  return true;
}

bool IsHiddenPathComponent(const std::string& name)
{
  const auto pos = name.find_last_of("/");
  const std::string base =
      (pos == std::string::npos) ? name : name.substr(pos + 1U);
  return !base.empty() && base[0] == '.';
}

bool IsMacOsResourceFolder(const std::string& name)
{
  // Typical macOS resource folder inside ZIPs: "__MACOSX/" at the root or
  // nested as a path component. We treat any path that starts with
  // "__MACOSX/" or contains "/__MACOSX/" as a resource folder entry that
  // should be ignored.
  if (name == "__MACOSX" || name.rfind("__MACOSX/", 0U) == 0U) {
    return true;
  }

  const auto pos = name.find("/__MACOSX/");
  return pos != std::string::npos;
}

bool IsMacOsMetadataFile(const std::string& name)
{
  const auto pos = name.find_last_of('/');
  const std::string base =
      (pos == std::string::npos) ? name : name.substr(pos + 1U);

  // Explicitly skip Finder metadata files commonly present in ZIPs
  // created on macOS.
  return base == ".DS_Store" || base == "._.DS_Store";
}

bool FilesAreIdentical(const fs::path& existing,
                       const std::vector<unsigned char>& newData)
{
  std::error_code ec;
  const auto fileSize = fs::file_size(existing, ec);
  if (ec || fileSize != newData.size()) {
    return false;
  }

  std::ifstream in(existing, std::ios::binary);
  if (!in.is_open()) {
    return false;
  }

  constexpr std::size_t kChunkSize = 8192;
  std::vector<unsigned char> buffer;
  buffer.resize(kChunkSize);

  std::size_t offset = 0;
  while (offset < newData.size()) {
    const std::size_t toRead =
        std::min(kChunkSize, newData.size() - offset);
    in.read(reinterpret_cast<char*>(buffer.data()),
            static_cast<std::streamsize>(toRead));
    const auto got = static_cast<std::size_t>(in.gcount());
    if (got != toRead) {
      return false;
    }
    if (!std::equal(newData.begin() + static_cast<std::ptrdiff_t>(offset),
                    newData.begin() + static_cast<std::ptrdiff_t>(offset + toRead),
                    buffer.begin())) {
      return false;
    }
    offset += toRead;
  }

  return true;
}

#if defined(SOUNDTABLE_HAVE_LIBZIP)

bool ExtractEntryToMemory(zip_t* archive,
                          zip_uint64_t index,
                          std::vector<unsigned char>& outData,
                          std::string& outError)
{
  zip_stat_t st;
  if (zip_stat_index(archive, index, 0, &st) != 0) {
    outError = "Failed to stat ZIP entry";
    return false;
  }

  if (st.size == 0) {
    outData.clear();
    return true;
  }

  zip_file_t* file = zip_fopen_index(archive, index, 0);
  if (file == nullptr) {
    outError = "Failed to open ZIP entry";
    return false;
  }

  outData.resize(static_cast<std::size_t>(st.size));
  std::size_t offset = 0;

  while (offset < outData.size()) {
    const zip_uint64_t remaining =
        static_cast<zip_uint64_t>(outData.size() - offset);
    const zip_int64_t n = zip_fread(
        file, outData.data() + static_cast<std::ptrdiff_t>(offset),
        remaining);
    if (n < 0) {
      outError = "Failed to read ZIP entry";
      zip_fclose(file);
      return false;
    }
    if (n == 0) {
      break;
    }
    offset += static_cast<std::size_t>(n);
  }

  zip_fclose(file);

  if (offset != outData.size()) {
    outError = "Truncated ZIP entry";
    return false;
  }

  return true;
}

bool WriteFilePossiblySkippingIdentical(const fs::path& destPath,
                                        const std::vector<unsigned char>& data,
                                        std::string& outError)
{
  std::error_code ec;
  const auto parent = destPath.parent_path();
  if (!parent.empty()) {
    fs::create_directories(parent, ec);
    if (ec) {
      outError = "Failed to create directory: " + parent.string();
      return false;
    }
  }

  if (fs::exists(destPath, ec) && !ec &&
      fs::is_regular_file(destPath, ec) && !ec) {
    if (FilesAreIdentical(destPath, data)) {
      // File already exists and is byte‑for‑byte identical: keep it.
      return true;
    }
  }

  std::ofstream out(destPath, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    outError = "Failed to write file: " + destPath.string();
    return false;
  }

  if (!data.empty()) {
    out.write(reinterpret_cast<const char*>(data.data()),
              static_cast<std::streamsize>(data.size()));
    if (!out.good()) {
      outError = "Failed to flush file: " + destPath.string();
      return false;
    }
  }

  return true;
}

#endif  // SOUNDTABLE_HAVE_LIBZIP

}  // namespace

bool LoadReactableSessionFromRtz(const std::string& rtz_path,
                                 const std::string& com_reactable_root,
                                 Scene& scene,
                                 ReactablePatchMetadata* const metadata,
                                 std::string* const error_message)
{
#if !defined(SOUNDTABLE_HAVE_LIBZIP)
  if (error_message != nullptr) {
    *error_message =
        "RTZ support is not available (libzip not enabled at build time)";
  }
  (void)rtz_path;
  (void)com_reactable_root;
  (void)scene;
  (void)metadata;
  return false;
#else
  if (metadata != nullptr) {
    metadata->author_name.clear();
    metadata->patch_name.clear();
    metadata->master_colour_argb = 0xFFFFFFFFU;
    metadata->master_muted = false;
  }

  const fs::path rtzPath(rtz_path);
  if (!fs::exists(rtzPath)) {
    if (error_message != nullptr) {
      *error_message = "RTZ file not found: " + rtzPath.string();
    }
    return false;
  }

  if (!HasZipSignature(rtzPath)) {
    if (error_message != nullptr) {
      *error_message = "RTZ file is not a valid ZIP archive";
    }
    return false;
  }

  int zipErr = 0;
  zip_t* archive = zip_open(rtzPath.string().c_str(), ZIP_RDONLY, &zipErr);
  if (archive == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Failed to open RTZ archive";
    }
    return false;
  }

  std::string rtpEntryName;
  zip_uint64_t rtpEntryIndex = 0;

  // Track .rtp candidates both at the root of the archive and at any
  // depth. Prefer a unique root‑level .rtp when present; otherwise, if
  // there is exactly one .rtp anywhere, accept it as the session file.
  int rootRtpCount = 0;
  std::string rootRtpName;
  zip_uint64_t rootRtpIndex = 0;

  struct RtpCandidate {
    std::string name;
    zip_uint64_t index;
  };
  std::vector<RtpCandidate> allRtpEntries;

  const zip_int64_t numEntries = zip_get_num_entries(archive, 0);
  for (zip_uint64_t i = 0; i < static_cast<zip_uint64_t>(numEntries); ++i) {
    const char* nameCStr = zip_get_name(archive, i, 0);
    if (nameCStr == nullptr) {
      continue;
    }
    const std::string name{nameCStr};

    if (IsMacOsResourceFolder(name) || IsMacOsMetadataFile(name)) {
      // Ignore macOS resource/metadata entries such as __MACOSX/ and
      // .DS_Store / ._.DS_Store when scanning for .rtp files.
      continue;
    }

    const std::string lower = ToLower(name);
    if (lower.size() >= 4U &&
        lower.compare(lower.size() - 4U, 4U, ".rtp") == 0) {
      allRtpEntries.push_back(RtpCandidate{name, i});

      const bool isRootLevel = name.find('/') == std::string::npos;
      if (isRootLevel) {
        ++rootRtpCount;
        if (rootRtpCount == 1) {
          rootRtpName = name;
          rootRtpIndex = i;
        }
      }
    }
  }

  if (rootRtpCount > 1) {
    zip_close(archive);
    if (error_message != nullptr) {
      *error_message =
          "RTZ archive contains multiple root‑level .rtp files";
    }
    return false;
  }

  if (rootRtpCount == 1) {
    rtpEntryName = rootRtpName;
    rtpEntryIndex = rootRtpIndex;
  } else {
    if (allRtpEntries.empty()) {
      zip_close(archive);
      if (error_message != nullptr) {
        *error_message = "RTZ archive does not contain a .rtp file";
      }
      return false;
    }
    if (allRtpEntries.size() > 1U) {
      zip_close(archive);
      if (error_message != nullptr) {
        *error_message =
            "RTZ archive contains multiple .rtp files";
      }
      return false;
    }

    rtpEntryName = allRtpEntries[0].name;
    rtpEntryIndex = allRtpEntries[0].index;
  }

  const std::string rtpLower = ToLower(rtpEntryName);
  const std::size_t dotPos = rtpLower.rfind('.');
  const std::string rtpBaseName =
      (dotPos == std::string::npos) ? rtpEntryName
                                     : rtpEntryName.substr(0U, dotPos);

  fs::path comRoot = com_reactable_root.empty()
                         ? fs::path("com.reactable")
                         : fs::path(com_reactable_root);

  std::error_code ec;
  fs::create_directories(comRoot, ec);
  ec.clear();

  const fs::path sessionsDir = comRoot / "Sessions";
  const fs::path samplesDir = comRoot / "Samples";
  fs::create_directories(sessionsDir, ec);
  ec.clear();
  fs::create_directories(samplesDir, ec);
  ec.clear();

  // 1) Extract the .rtp into com.reactable/Sessions/.
  std::vector<unsigned char> rtpData;
  std::string ioError;
  if (!ExtractEntryToMemory(archive, rtpEntryIndex, rtpData, ioError)) {
    zip_close(archive);
    if (error_message != nullptr) {
      *error_message = ioError;
    }
    return false;
  }

  const fs::path rtpDest = sessionsDir / rtpEntryName;
  if (!WriteFilePossiblySkippingIdentical(rtpDest, rtpData, ioError)) {
    zip_close(archive);
    if (error_message != nullptr) {
      *error_message = ioError;
    }
    return false;
  }

  // 2) Extract optional sample folder that matches the .rtp basename.
  const std::string baseFolderPrefix = rtpBaseName + "/";

  for (zip_uint64_t i = 0; i < static_cast<zip_uint64_t>(numEntries); ++i) {
    const char* nameCStr = zip_get_name(archive, i, 0);
    if (nameCStr == nullptr) {
      continue;
    }
    const std::string entryName{nameCStr};

    if (entryName == rtpEntryName) {
      continue;  // already handled.
    }

    // We only care about entries under the folder whose name matches
    // the .rtp basename, e.g. "Loopdemo/..." for Loopdemo.rtp.
    if (entryName.rfind(baseFolderPrefix, 0U) != 0U) {
      continue;
    }

    if (entryName.back() == '/') {
      // Directory entry; it'll be created implicitly when extracting
      // files.
      continue;
    }

    if (IsMacOsResourceFolder(entryName) || IsMacOsMetadataFile(entryName)) {
      // Skip macOS resource folders and metadata files explicitly.
      continue;
    }

    if (IsHiddenPathComponent(entryName)) {
      // Skip hidden/temporary files such as .DS_Store or ._prefix.
      continue;
    }

    std::vector<unsigned char> data;
    if (!ExtractEntryToMemory(archive, i, data, ioError)) {
      zip_close(archive);
      if (error_message != nullptr) {
        *error_message = ioError;
      }
      return false;
    }

    // Preserve the folder name as is under com.reactable/Samples/.
    const std::string relativeSubPath =
        entryName.substr(baseFolderPrefix.size());
    const fs::path dest = samplesDir / rtpBaseName / relativeSubPath;

    if (!WriteFilePossiblySkippingIdentical(dest, data, ioError)) {
      zip_close(archive);
      if (error_message != nullptr) {
        *error_message = ioError;
      }
      return false;
    }
  }

  zip_close(archive);

  // 3) Finally, load the extracted .rtp and validate that it is a
  //    proper Reactable session.
  std::string loadError;
  const bool ok = LoadReactablePatchFromFile(
      rtpDest.string(), scene, metadata, &loadError);
  if (!ok) {
    if (error_message != nullptr) {
      *error_message =
          "Failed to load .rtp from RTZ archive: " + loadError;
    }
    return false;
  }

  if (error_message != nullptr) {
    error_message->clear();
  }

  return true;
#endif  // SOUNDTABLE_HAVE_LIBZIP
}

}  // namespace soundtable
