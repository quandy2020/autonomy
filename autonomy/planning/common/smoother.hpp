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

#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/port.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace planning {
namespace common {

/**
 * @class Smoother
 * @brief smoother interface that acts as a virtual base class for all smoother plugins
 */
class Smoother
{
public:
  using Ptr = std::shared_ptr<nav2_core::Smoother>;
    /**
     * Define PlannerInterface::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(PlannerInterface)

    /**
     * @brief Virtual destructor
     */
    virtual ~Smoother() {}

    // virtual void configure(
    //     const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    //     std::string name, std::shared_ptr<tf2_ros::Buffer>,
    //     std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    //     std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) = 0;
  
    /**
     * @brief Method to smooth given path
     *
     * @param path In-out path to be smoothed
     * @param max_time Maximum duration smoothing should take
     * @return If smoothing was completed (true) or interrupted by time limit (false)
     */
    virtual bool Smooth(
        commsgs::planning_msgs::Path& path,
        const autonomy::common::Duration& max_time) = 0;
};

}  // namespace common
}  // namespace planning
}  // namespace autonomy
