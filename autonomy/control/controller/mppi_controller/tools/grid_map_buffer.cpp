/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/control/controller/mppi_controller/tools/grid_map_buffer.hpp"

#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace tools {

bool GridMapBuffer::Update(const automsgs::msgs::map_msgs::GridMap& message) {
    grid_map::GridMap map;
    if (!grid_map::GridMapConverter::fromMessage(message, map)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    map_ = std::move(map);
    return true;
}

bool GridMapBuffer::Copy(grid_map::GridMap* out) const {
    if (out == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!map_.has_value()) {
        return false;
    }
    *out = *map_;
    return true;
}

bool GridMapBuffer::HasMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_.has_value();
}

}  // namespace tools
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
