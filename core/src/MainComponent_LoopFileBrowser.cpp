#include "MainComponent_LoopFileBrowser.h"

#include "MainComponent_TextScroll.h"
#include "core/AudioModules.h"

LoopFileBrowser::LoopFileBrowser(
    StateMap& stateMap,
    ListMap& listMap,
    juce::File samplesRootDir,
    std::function<void()> repaintCallback,
    std::function<void(const std::string&)> markLabelActive,
    std::function<soundtable::LoopModule*(const std::string&)> findLoopModule,
    std::function<bool(const std::string& moduleId,
                       int slotIndex,
                       const std::string& fullPath,
                       float beats,
                       std::string* error)>
                loadSampleFn,
        std::function<int(const std::string& moduleId, int slotIndex)>
                queryBeatsFn)
    : stateMap_(stateMap),
      listMap_(listMap),
      samplesRootDir_(std::move(samplesRootDir)),
      repaintCallback_(std::move(repaintCallback)),
      markLabelActive_(std::move(markLabelActive)),
      findLoopModule_(std::move(findLoopModule)),
            loadSampleFn_(std::move(loadSampleFn)),
            queryBeatsFn_(std::move(queryBeatsFn))
{
}

int LoopFileBrowser::computeSlotIndexFromSampleParam(float sampleParam)
{
    sampleParam = juce::jlimit(0.0F, 1.0F, sampleParam);
    int index = static_cast<int>(sampleParam * 4.0F);
    if (index < 0) {
        index = 0;
    } else if (index > 3) {
        index = 3;
    }
    return index;
}

soundtable::ui::TextScrollList*
LoopFileBrowser::getOrCreateList(const std::string& moduleId)
{
    auto it = listMap_.find(moduleId);
    if (it != listMap_.end() && it->second != nullptr) {
        return it->second.get();
    }

    auto list = std::make_unique<soundtable::ui::TextScrollList>();
    list->setMaxVisibleItems(6);
    list->setRowHeight(18.0F);

    soundtable::ui::TextScrollList* raw = list.get();
    raw->setOnSelectionChanged(
        [this, moduleId](int index) {
            handleSelectionChanged(moduleId, index);
        });

    listMap_.emplace(moduleId, std::move(list));
    return raw;
}

void LoopFileBrowser::ensureInitialised(const std::string& moduleId,
                                        soundtable::LoopModule* loopModule)
{
    auto& state = stateMap_[moduleId];
    if (!state.initialised) {
        state.rootDir = samplesRootDir_;

        juce::File initialDir = state.rootDir;

        if (loopModule != nullptr) {
            const auto& loops = loopModule->loops();
            if (!loops.empty()) {
                float sampleParam =
                    loopModule->GetParameterOrDefault("sample", 0.0F);
                int selectedIndex =
                    computeSlotIndexFromSampleParam(sampleParam);
                if (selectedIndex >=
                    static_cast<int>(loops.size())) {
                    selectedIndex = 0;
                }

                const auto& def =
                    loops[static_cast<std::size_t>(selectedIndex)];
                if (!def.filename.empty()) {
                    juce::File f = state.rootDir.getChildFile(
                        def.filename);
                    juce::File parent = f.getParentDirectory();
                    if (parent == state.rootDir ||
                        parent.isAChildOf(state.rootDir)) {
                        initialDir = parent;
                    }
                    state.selectedFile = f;
                }
            }
        }

        state.currentDir = initialDir;
        state.initialised = true;
    } else if (loopModule != nullptr) {
        // Keep the selected file in sync with the currently active
        // Loop slot, so external changes (e.g. via the side bar or
        // rotation gestures) are reflected when rebuilding the list.
        const auto& loops = loopModule->loops();
        state.selectedFile = juce::File();
        if (!loops.empty()) {
            float sampleParam =
                loopModule->GetParameterOrDefault("sample", 0.0F);
            int selectedIndex =
                computeSlotIndexFromSampleParam(sampleParam);
            if (selectedIndex >=
                static_cast<int>(loops.size())) {
                selectedIndex = 0;
            }

            const auto& def =
                loops[static_cast<std::size_t>(selectedIndex)];
            if (!def.filename.empty()) {
                state.selectedFile =
                    state.rootDir.getChildFile(def.filename);
            }
        }
    }
}

void LoopFileBrowser::rebuildEntries(const std::string& moduleId,
                                     soundtable::LoopModule* loopModule)
{
    juce::ignoreUnused(loopModule);

    auto it = stateMap_.find(moduleId);
    if (it == stateMap_.end()) {
        return;
    }

    auto& state = it->second;
    state.entries.clear();

    const bool hasRootDir = state.rootDir.isDirectory();

    // Always expose the clear-sample pseudo-entry as the first row.
    Entry clearEntry;
    clearEntry.kind = EntryKind::kClearSample;
    state.entries.push_back(clearEntry);

    const bool hasSelectedFile =
        !state.selectedFile.getFullPathName().isEmpty();
    int selectedRow = hasSelectedFile ? -1 : 0;

    // Second row: optional parent folder when we are below the root.
    if (hasRootDir && state.currentDir != state.rootDir &&
        state.currentDir.isDirectory() &&
        (state.currentDir.isAChildOf(state.rootDir) ||
         state.currentDir == state.rootDir)) {
        juce::File parent = state.currentDir.getParentDirectory();
        if (parent == state.rootDir ||
            parent.isAChildOf(state.rootDir)) {
            Entry parentEntry;
            parentEntry.kind = EntryKind::kParentFolder;
            parentEntry.file = parent;
            state.entries.push_back(parentEntry);
        }
    }

    // Collect directories and audio files in the current directory.
    juce::Array<juce::File> dirs;
    juce::Array<juce::File> files;
    if (state.currentDir.isDirectory()) {
        state.currentDir.findChildFiles(dirs,
                                        juce::File::findDirectories,
                                        false);
        state.currentDir.findChildFiles(files,
                                        juce::File::findFiles,
                                        false);
    }

    auto byName = [](const juce::File& a, const juce::File& b) {
        return a.getFileName().compareIgnoreCase(b.getFileName()) < 0;
    };
    std::sort(dirs.begin(), dirs.end(), byName);
    std::sort(files.begin(), files.end(), byName);

    const auto selectedPath =
        state.selectedFile.getFullPathName();

    // Directories first, in yellow.
    for (int i = 0; i < dirs.size(); ++i) {
        Entry entry;
        entry.kind = EntryKind::kDirectory;
        entry.file = dirs.getReference(i);
        state.entries.push_back(entry);

        const int rowIndex = static_cast<int>(state.entries.size()) - 1;
        if (hasSelectedFile &&
            entry.file.getFullPathName() == selectedPath) {
            selectedRow = rowIndex;
        }
    }

    // Audio files supported by the Loop loader.
    auto isSupportedAudioFile = [](const juce::File& f) {
        const juce::String ext = f.getFileExtension().toLowerCase();
        return ext == ".wav" || ext == ".flac" || ext == ".ogg" ||
               ext == ".opus";
    };

    for (int i = 0; i < files.size(); ++i) {
        const juce::File& f = files.getReference(i);
        if (!isSupportedAudioFile(f)) {
            continue;
        }

        Entry entry;
        entry.kind = EntryKind::kFile;
        entry.file = f;
        state.entries.push_back(entry);

        const int rowIndex = static_cast<int>(state.entries.size()) - 1;
        if (hasSelectedFile &&
            entry.file.getFullPathName() == selectedPath) {
            selectedRow = rowIndex;
        }
    }

    // Build TextScrollList items mirroring the entry vector.
    auto* list = getOrCreateList(moduleId);
    if (list == nullptr) {
        return;
    }

    std::vector<soundtable::ui::TextScrollList::Item> items;
    items.reserve(state.entries.size());

    for (std::size_t i = 0; i < state.entries.size(); ++i) {
        const auto& entry = state.entries[i];
        soundtable::ui::TextScrollList::Item item;

        switch (entry.kind) {
        case EntryKind::kClearSample:
            item.text = "[ CLEAR SAMPLE ]";
            item.style.useCustomColour = true;
            item.style.colour = juce::Colours::skyblue;
            item.style.bold = true;
            break;
        case EntryKind::kParentFolder:
            item.text = "< Parent Folder";
            item.style.useCustomColour = true;
            item.style.colour = juce::Colours::lightgreen;
            item.style.bold = true;
            break;
        case EntryKind::kDirectory:
            item.text = entry.file.getFileName();
            item.style.useCustomColour = true;
            item.style.colour = juce::Colours::yellow;
            item.style.bold = false;
            break;
        case EntryKind::kFile:
        default:
            item.text = entry.file.getFileName();
            item.style.useCustomColour = false;
            item.style.bold = false;
            break;
        }

        items.push_back(std::move(item));
    }

    list->setItems(std::move(items));
    if (selectedRow >= 0 &&
        selectedRow < static_cast<int>(state.entries.size())) {
        list->setSelectedIndex(selectedRow, false);
    } else {
        list->setSelectedIndex(-1, false);
    }
}

void LoopFileBrowser::handleSelectionChanged(const std::string& moduleId,
                                             const int rowIndex)
{
    auto itState = stateMap_.find(moduleId);
    if (itState == stateMap_.end()) {
        return;
    }

    auto& state = itState->second;
    if (rowIndex < 0 ||
        rowIndex >= static_cast<int>(state.entries.size())) {
        return;
    }

    auto* loopModule = findLoopModule_(moduleId);
    if (loopModule == nullptr) {
        return;
    }

    const auto& entry = state.entries[static_cast<std::size_t>(rowIndex)];

    // Determine the currently selected Loop slot index so that file
    // operations apply to the active segment on the left bar.
    float sampleParam = loopModule->GetParameterOrDefault("sample", 0.0F);
    int slotIndex = computeSlotIndexFromSampleParam(sampleParam);

    auto& loops = loopModule->mutable_loops();
    if (slotIndex < 0) {
        slotIndex = 0;
    }
    if (slotIndex >= static_cast<int>(loops.size())) {
        loops.resize(static_cast<std::size_t>(slotIndex + 1));
    }

    soundtable::LoopDefinition& def =
        loops[static_cast<std::size_t>(slotIndex)];

    switch (entry.kind) {
    case EntryKind::kClearSample: {
        def.filename.clear();
        state.selectedFile = juce::File();

        // Clearing the sample leaves the engine slot empty; the audio
        // callback will skip it as there is no decoded buffer.
        std::string error;
        juce::ignoreUnused(error);

        markLabelActive_(moduleId);
        repaintCallback_();
        break;
    }
    case EntryKind::kParentFolder: {
        if (entry.file == juce::File()) {
            break;
        }

        juce::File parent = entry.file;
        if (parent == state.rootDir ||
            parent.isAChildOf(state.rootDir)) {
            state.currentDir = parent;
        } else {
            state.currentDir = state.rootDir;
        }
        rebuildEntries(moduleId, loopModule);
        repaintCallback_();
        break;
    }
    case EntryKind::kDirectory: {
        if (entry.file.isDirectory() &&
            (entry.file == state.rootDir ||
             entry.file.isAChildOf(state.rootDir))) {
            state.currentDir = entry.file;
            rebuildEntries(moduleId, loopModule);
            repaintCallback_();
        }
        break;
    }
    case EntryKind::kFile:
    default: {
        if (!entry.file.existsAsFile()) {
            break;
        }

        const juce::String fullPath =
            entry.file.getFullPathName();
        const juce::String rootPath =
            state.rootDir.getFullPathName();

        juce::String relative;
        if (fullPath.startsWith(rootPath)) {
            const int cut = rootPath.length();
            if (fullPath.length() > cut) {
                // Skip leading path separator when present.
                const juce::juce_wchar sep =
                    juce::File::getSeparatorChar();
                const bool hasSep =
                    fullPath[cut] == sep;
                relative = hasSep ? fullPath.substring(cut + 1)
                                   : fullPath.substring(cut);
            }
        } else {
            relative = fullPath;
        }

        def.filename = relative.toStdString();
        state.selectedFile = entry.file;

        // When selecting a new sample from the browser we always ask
        // the audio engine to auto-detect the beat count from the
        // decoded audio instead of reusing the previous metadata from
        // the .rtp file. Passing beats <= 0 triggers estimation based
        // on the file duration and current global Loop tempo.
        std::string error;
        constexpr float kAutoDetectBeats = -1.0F;
        const bool ok = loadSampleFn_(
            moduleId, slotIndex, fullPath.toStdString(),
            kAutoDetectBeats, &error);
        if (!ok) {
            juce::Logger::writeToLog(
                juce::String("[soundtable-core] Loop: failed to load "
                             "sample from browser: ") +
                fullPath + " (" + juce::String(error) + ")");
        } else {
            // If the audio engine successfully loaded the sample,
            // query the effective beat count actually used for this
            // slot (taking into account .rtp metadata, cached values
            // and auto-detection) and propagate it back into the
            // LoopDefinition so that UI elements such as the
            // playhead bar remain in sync with the audio.
            if (queryBeatsFn_) {
                const int effectiveBeats =
                    queryBeatsFn_(moduleId, slotIndex);
                if (effectiveBeats > 0) {
                    def.beats = effectiveBeats;
                }
            }
            markLabelActive_(moduleId);
        }

        repaintCallback_();
        break;
    }
    }
}
