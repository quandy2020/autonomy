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

#include <functional>
#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/port.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/planning/proto/planning_options.pb.h"

namespace autonomy {
namespace map {
namespace costmap_2d {
class Costmap2DWrapper;
}  // namespace costmap_2d
}  // namespace map
namespace transform {
class Buffer;
}  // namespace transform
}  // namespace autonomy

namespace autonomy {
namespace planning {
namespace common {

class GlobalPlanner
{
public:
    using TfBuffer = autonomy::transform::Buffer;

    AUTONOMY_SMART_PTR_DEFINITIONS(GlobalPlanner)

    virtual ~GlobalPlanner() = default;

    const proto::PlannerOptions& GetOptions() const { return options_; }
    const std::string& GetName() const { return name_; }
    map::costmap_2d::Costmap2DWrapper* GetCostmapWrapper() const {
        return costmap_.get();
    }

    virtual uint32 CreatePlan(const automsgs::msgs::geometry_msgs::PoseStamped& start,
                              const automsgs::msgs::geometry_msgs::PoseStamped& goal,
                              automsgs::msgs::planning_msgs::Path& plan,
                              std::function<bool()> cancel_checker) = 0;

protected:
    GlobalPlanner() = default;

    GlobalPlanner(const proto::PlannerOptions& options, std::string name,
                  std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap);

    proto::PlannerOptions options_;
    std::string name_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
};

}  // namespace common
}  // namespace planning
}  // namespace autonomy
