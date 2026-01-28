#pragma once

#include <vector>

namespace soundtable::tracker {

struct TrackedObject {
    int id{};
    float x_norm{};
    float y_norm{};
    float angle_rad{};
    bool alive{true};
};

using TrackedObjectList = std::vector<TrackedObject>;

} // namespace soundtable::tracker
