#pragma once

#include <unordered_map>
#include <vector>

#include "TrackerTypes.h"

namespace soundtable::tracker {

class TrackerState {
public:
    TrackerState() = default;

    void update(const TrackedObjectList& objects);

    [[nodiscard]] std::vector<int> collectRemovals(int maxMissingFrames);

private:
    std::unordered_map<int, int> missingFramesById_;
};

} // namespace soundtable::tracker
