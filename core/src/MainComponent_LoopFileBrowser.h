#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <juce_gui_basics/juce_gui_basics.h>

#include "MainComponent_TextScroll.h"

namespace soundtable {
class LoopModule;
}  // namespace soundtable

// Lightweight per-module file browser helper for Loop modules. It
// owns the directory traversal state and drives a TextScrollList with
// entries representing clear/parent/directory/file rows.
class LoopFileBrowser {
public:
    enum class EntryKind {
        kClearSample = 0,
        kParentFolder,
        kDirectory,
        kFile,
    };

    struct Entry {
        EntryKind kind{EntryKind::kFile};
        juce::File file;  // For directories, parent folder and files.
    };

    struct State {
        juce::File rootDir;      // com.reactable/Samples
        juce::File currentDir;   // Directory currently listed.
        juce::File selectedFile; // Absolute path of active sample file.
        std::vector<Entry> entries;
        bool initialised{false};
    };

    using StateMap =
        std::unordered_map<std::string, State>;

    using ListMap = std::unordered_map<
        std::string,
        std::unique_ptr<soundtable::ui::TextScrollList>>;

    LoopFileBrowser(StateMap& stateMap,
                    ListMap& listMap,
                    juce::File samplesRootDir,
                    std::function<void()> repaintCallback,
                    std::function<void(const std::string&)> markLabelActive,
                    std::function<soundtable::LoopModule*(const std::string&)>
                        findLoopModule,
                    std::function<bool(const std::string& moduleId,
                                       int slotIndex,
                                       const std::string& fullPath,
                                       float beats,
                                       std::string* error)>
                        loadSampleFn,
                    std::function<int(const std::string& moduleId,
                                       int slotIndex)>
                        queryBeatsFn);

    // Ensure that the browser state for a given module id is
    // initialised and synchronised with the currently active Loop
    // slot of the provided LoopModule.
    void ensureInitialised(const std::string& moduleId,
                           soundtable::LoopModule* loopModule);

    // Rebuild the entry list and associated TextScrollList items for
    // a given module.
    void rebuildEntries(const std::string& moduleId,
                        soundtable::LoopModule* loopModule);

    // Retrieve or lazily create the TextScrollList associated with a
    // given module id.
    [[nodiscard]] soundtable::ui::TextScrollList*
    getOrCreateList(const std::string& moduleId);

    // Handle a selection change coming from the TextScrollList for a
    // given module id.
    void handleSelectionChanged(const std::string& moduleId,
                                int rowIndex);

private:
    static int computeSlotIndexFromSampleParam(float sampleParam);

    StateMap& stateMap_;
    ListMap& listMap_;
    juce::File samplesRootDir_;
    std::function<void()> repaintCallback_;
    std::function<void(const std::string&)> markLabelActive_;
    std::function<soundtable::LoopModule*(const std::string&)> findLoopModule_;
    std::function<bool(const std::string& moduleId,
                       int slotIndex,
                       const std::string& fullPath,
                       float beats,
                       std::string* error)>
        loadSampleFn_;
    std::function<int(const std::string& moduleId, int slotIndex)>
        queryBeatsFn_;
};
