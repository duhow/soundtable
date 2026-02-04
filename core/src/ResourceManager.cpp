#include "ResourceManager.h"

#include <cstdlib>
#include <iostream>
#include <zip.h>

namespace soundtable {

bool ResourceManager::initialize()
{
    if (initialized_) {
        return true;
    }

    // Determine the resource directory using XDG Base Directory specification.
    // Default: $HOME/.local/share/soundtable
    const char* xdgDataHome = std::getenv("XDG_DATA_HOME");
    const char* home = std::getenv("HOME");

    if (xdgDataHome && xdgDataHome[0] != '\0') {
        resourceDir_ = std::string(xdgDataHome) + "/soundtable";
    } else if (home && home[0] != '\0') {
        resourceDir_ = std::string(home) + "/.local/share/soundtable";
    } else {
        // Fallback to current directory if no HOME set (unlikely).
        resourceDir_ = "./soundtable";
    }

    juce::Logger::writeToLog("[ResourceManager] Resource directory: " + juce::String(resourceDir_));

    // Create the resource directory if it doesn't exist.
    const juce::File resourceDir(resourceDir_);
    if (!resourceDir.isDirectory()) {
        const auto createResult = resourceDir.createDirectory();
        if (!createResult.wasOk()) {
            juce::Logger::writeToLog(
                "[ResourceManager] Failed to create resource directory: " +
                createResult.getErrorMessage());
            return false;
        }
        juce::Logger::writeToLog("[ResourceManager] Created resource directory");
    }

    // Check if resources have already been extracted.
    if (hasExtractionMarker()) {
        juce::Logger::writeToLog("[ResourceManager] Extraction marker found, skipping ZIP extraction");
        initialized_ = true;
        return true;
    }

    // Search for reactableresources.zip in predefined locations.
    const juce::File zipFile = findResourcesZip();
    if (!zipFile.existsAsFile()) {
        juce::Logger::writeToLog(
            "[ResourceManager] reactableresources.zip not found in predefined locations. "
            "Resources may not be available.");
        // This is not a critical error; the application can still run with resources
        // loaded from the build output or embedded data.
        initialized_ = true;
        return true;
    }

    juce::Logger::writeToLog("[ResourceManager] Found reactableresources.zip at: " +
                            zipFile.getFullPathName());

    // Attempt to extract the ZIP.
    if (!extractResourcesZip(zipFile)) {
        juce::Logger::writeToLog("[ResourceManager] ZIP extraction failed");
        return false;
    }

    initialized_ = true;
    return true;
}

juce::File ResourceManager::getResourceFile(const juce::String& relativePath) const
{
    if (!initialized_ || resourceDir_.empty()) {
        return juce::File();
    }

    const juce::File baseDir(resourceDir_);
    return baseDir.getChildFile(relativePath);
}

juce::File ResourceManager::findResourcesZip() const
{
    static constexpr const char* kZipFilename = "reactableresources.zip";

    const juce::File exeDir = juce::File::getSpecialLocation(
        juce::File::currentExecutableFile)
        .getParentDirectory();

    std::vector<juce::File> searchDirs;
    searchDirs.reserve(12);

    // Relative to current working directory.
    const juce::File cwd = juce::File::getCurrentWorkingDirectory();
    searchDirs.push_back(cwd);
    searchDirs.push_back(cwd.getChildFile(".."));
    searchDirs.push_back(cwd.getChildFile("../.."));

    // Relative to user data directory.
    searchDirs.push_back(juce::File(resourceDir_ + "/.."));

    // Relative to executable directory.
    searchDirs.push_back(exeDir);
    searchDirs.push_back(exeDir.getChildFile(".."));
    searchDirs.push_back(exeDir.getChildFile("../.."));
    searchDirs.push_back(exeDir.getChildFile("../resources"));

    // System-wide resource locations.
    searchDirs.push_back(juce::File("/usr/share/soundtable"));

    // Snap installations.
    if (const char* snapEnv = std::getenv("SNAP")) {
        juce::File snapRoot(snapEnv);
        searchDirs.push_back(
            snapRoot.getChildFile("share").getChildFile("soundtable"));
    }

    // AppImage root.
    if (const char* appDirEnv = std::getenv("APPDIR")) {
        juce::File appRoot(appDirEnv);
        searchDirs.push_back(appRoot.getChildFile("resources"));
    }

    // Search for the ZIP file in all directories.
    for (const auto& dir : searchDirs) {
        const juce::File candidate = dir.getChildFile(kZipFilename);
        if (candidate.existsAsFile()) {
            return candidate;
        }
    }

    return juce::File();
}

bool ResourceManager::extractResourcesZip(const juce::File& zipFile)
{
    // Open the ZIP file using libzip.
    int zipError = 0;
    zip_t* zipPtr = zip_open(zipFile.getFullPathName().toRawUTF8(), 0, &zipError);

    if (!zipPtr) {
        zip_error_t error;
        zip_error_init_with_code(&error, zipError);
        juce::Logger::writeToLog(
            "[ResourceManager] Failed to open ZIP file: " +
            juce::String(zip_error_strerror(&error)));
        zip_error_fini(&error);
        return false;
    }

    // Extract all files from the ZIP to the resource directory.
    const juce::String resourcePath(resourceDir_);
    const zip_uint64_t numEntries = zip_get_num_entries(zipPtr, 0);

    for (zip_uint64_t i = 0; i < numEntries; ++i) {
        zip_stat_t stat;
        zip_stat_index(zipPtr, i, 0, &stat);

        const juce::String entryName(stat.name);
        const juce::File targetFile = juce::File(resourcePath).getChildFile(entryName);

        // If the entry is a directory (ends with /), create it.
        if (entryName.endsWithChar('/')) {
            const auto createDirResult = targetFile.createDirectory();
            if (!createDirResult.wasOk() && !targetFile.isDirectory()) {
                juce::Logger::writeToLog(
                    "[ResourceManager] Failed to create directory: " +
                    createDirResult.getErrorMessage());
                zip_close(zipPtr);
                return false;
            }
            continue;
        }

        // Create parent directories if needed.
        const juce::File parentDir = targetFile.getParentDirectory();
        if (!parentDir.isDirectory()) {
            const auto createDirResult = parentDir.createDirectory();
            if (!createDirResult.wasOk()) {
                juce::Logger::writeToLog(
                    "[ResourceManager] Failed to create parent directory: " +
                    createDirResult.getErrorMessage());
                zip_close(zipPtr);
                return false;
            }
        }

        // Extract the file.
        zip_file_t* filePtr = zip_fopen_index(zipPtr, i, 0);
        if (!filePtr) {
            juce::Logger::writeToLog(
                "[ResourceManager] Failed to open ZIP entry: " + entryName);
            zip_close(zipPtr);
            return false;
        }

        juce::MemoryBlock fileData;
        const int bufferSize = 16384;
        char buffer[bufferSize];
        zip_int64_t totalRead = 0;

        while (true) {
            const zip_int64_t bytesRead = zip_fread(filePtr, buffer, bufferSize);
            if (bytesRead < 0) {
                juce::Logger::writeToLog(
                    "[ResourceManager] Error reading ZIP entry: " + entryName);
                zip_fclose(filePtr);
                zip_close(zipPtr);
                return false;
            }
            if (bytesRead == 0) {
                break;
            }

            fileData.append(buffer, static_cast<size_t>(bytesRead));
            totalRead += bytesRead;
        }

        zip_fclose(filePtr);

        // Write the file to disk.
        if (!targetFile.replaceWithData(fileData.getData(), fileData.getSize())) {
            juce::Logger::writeToLog(
                "[ResourceManager] Failed to write file: " +
                targetFile.getFullPathName());
            zip_close(zipPtr);
            return false;
        }

        juce::Logger::writeToLog("[ResourceManager] Extracted: " + entryName);
    }

    zip_close(zipPtr);

    // Create the extraction marker.
    createExtractionMarker();
    juce::Logger::writeToLog("[ResourceManager] ZIP extraction completed successfully");

    return true;
}

void ResourceManager::createExtractionMarker() const
{
    const juce::File markerFile = juce::File(resourceDir_).getChildFile(".extracted");
    juce::FileOutputStream stream(markerFile);
    if (stream.openedOk()) {
        stream.writeText("true", false, false, nullptr);
        stream.flush();
    } else {
        juce::Logger::writeToLog(
            "[ResourceManager] Failed to create extraction marker file");
    }
}

bool ResourceManager::hasExtractionMarker() const
{
    const juce::File markerFile = juce::File(resourceDir_).getChildFile(".extracted");
    return markerFile.existsAsFile();
}

ResourceManager& ResourceManager::getInstance()
{
    static ResourceManager instance;
    return instance;
}

}  // namespace soundtable
