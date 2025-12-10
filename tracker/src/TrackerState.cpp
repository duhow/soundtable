#include "TrackerState.h"

namespace rectai::tracker {

void TrackerState::update(const TrackedObjectList& objects)
{
    // Marcar presentes como vistos este frame.
    for (const auto& obj : objects) {
        missingFramesById_[obj.id] = 0;
    }

    // Incrementar el contador de frames ausentes para los que no han aparecido.
    for (auto& entry : missingFramesById_) {
        const int id = entry.first;
        bool found = false;
        for (const auto& obj : objects) {
            if (obj.id == id) {
                found = true;
                break;
            }
        }
        if (!found) {
            ++entry.second;
        }
    }
}

std::vector<int> TrackerState::collectRemovals(const int maxMissingFrames)
{
    std::vector<int> removed;
    for (auto it = missingFramesById_.begin(); it != missingFramesById_.end();) {
        if (it->second >= maxMissingFrames) {
            removed.push_back(it->first);
            it = missingFramesById_.erase(it);
        } else {
            ++it;
        }
    }
    return removed;
}

} // namespace rectai::tracker
