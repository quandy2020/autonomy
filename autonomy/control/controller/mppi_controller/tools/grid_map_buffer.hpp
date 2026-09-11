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

#pragma once

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

#include <automsgs/msgs/map_msgs/grid_map.pb.h>

#include <mutex>
#include <optional>

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace tools {

/**
 * @brief Thread-safe latest GridMap snapshot for MPPI critics.
 *        Fed by ControllerServer from /perception/follow/grid (MoGe LocalGrid).
 */
class GridMapBuffer {
public:
    bool Update(const automsgs::msgs::map_msgs::GridMap& message);

    /** @return false if no map has been received yet. */
    bool Copy(grid_map::GridMap* out) const;

    bool HasMap() const;

private:
    mutable std::mutex mutex_;
    std::optional<grid_map::GridMap> map_;
};

}  // namespace tools
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
