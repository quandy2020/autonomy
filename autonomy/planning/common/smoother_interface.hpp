/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <chrono>
#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {
class Costmap2DWrapper;
}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy

namespace autonomy {
namespace planning {
namespace common {

/**
 * @class Smoother
 * @brief smoother interface that acts as a virtual base class for all smoother
 * plugins
 */
class Smoother
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Smoother)

    virtual ~Smoother() = default;

    const std::string& GetName() const { return name_; }

    map::costmap_2d::Costmap2DWrapper* GetCostmapWrapper() const {
        return costmap_wrapper_.get();
    }

    /**
     * @brief Method to smooth given path
     *
     * @param path In-out path to be smoothed
     * @param max_time Maximum duration smoothing should take
     * @return If smoothing was completed (true) or interrupted by time limit
     * (false)
     */
    virtual bool Smooth(commsgs::planning_msgs::Path& path,
                        const std::chrono::milliseconds& max_time) = 0;

protected:
    Smoother() = default;

    Smoother(std::string name,
             std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper);

    std::string name_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
};

}  // namespace common
}  // namespace planning
}  // namespace autonomy
