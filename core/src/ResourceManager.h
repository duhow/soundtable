#pragma once

#include <juce_core/juce_core.h>
#include <string>
#include <optional>

namespace soundtable {

/**
 * Manages resource loading from a user-data directory following XDG Base Directory specification.
 * 
 * On initialization, this manager:
 * 1. Determines the resource root directory: $HOME/.local/share/soundtable (via XDG_DATA_HOME)
 * 2. Looks for reactableresources.zip in predefined locations
 * 3. If found and not yet extracted, extracts it to the resource root
 * 4. Records extraction via a .extracted marker file to avoid re-extracting
 * 
 * This allows the application to work with bundled resources that are extracted on first run.
 */
class ResourceManager {
public:
    /**
     * Initialize the resource manager.
     * 
     * This will:
     * - Create the resource directory if it doesn't exist
     * - Search for reactableresources.zip in predefined locations
     * - Extract it if not already extracted
     * 
     * @return true if initialization succeeded; false if a critical error occurred
     *         (non-critical errors like missing ZIP are logged but don't fail initialization)
     */
    bool initialize();

    /**
     * Get the absolute path to the resource directory.
     * This is typically $HOME/.local/share/soundtable (or XDG_DATA_HOME/soundtable).
     * 
     * @return Resource directory path; empty string if not initialized
     */
    [[nodiscard]] const std::string& getResourceDir() const { return resourceDir_; }

    /**
     * Get a file within the resource directory.
     * 
     * @param relativePath Relative path within the resource directory (e.g., "Resources/atlas_2048.png")
     * @return juce::File pointing to the resource, may not exist
     */
    [[nodiscard]] juce::File getResourceFile(const juce::String& relativePath) const;

    /**
     * Check if the manager is initialized.
     */
    [[nodiscard]] bool isInitialized() const { return initialized_; }

    /**
     * Singleton accessor (optional, for convenience).
     * Note: Call initialize() on the singleton before using it.
     */
    static ResourceManager& getInstance();

private:
    ResourceManager() = default;

    // Search for reactableresources.zip in predefined locations.
    // Returns the file if found; otherwise an empty file.
    [[nodiscard]] juce::File findResourcesZip() const;

    // Extract reactableresources.zip to the resource directory.
    // Returns true if extraction succeeded or was already done (via .extracted marker).
    bool extractResourcesZip(const juce::File& zipFile);

    // Create a marker file to indicate successful extraction.
    void createExtractionMarker() const;

    // Check if extraction marker exists.
    [[nodiscard]] bool hasExtractionMarker() const;

    bool initialized_ = false;
    std::string resourceDir_;  // e.g., ~/.local/share/soundtable

    // Prevent copying
    ResourceManager(const ResourceManager&) = delete;
    ResourceManager& operator=(const ResourceManager&) = delete;
};

}  // namespace soundtable
