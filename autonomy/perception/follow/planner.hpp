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

#ifndef AUTONOMY_PERCEPTION_FOLLOW_PLANNER_HPP_
#define AUTONOMY_PERCEPTION_FOLLOW_PLANNER_HPP_

#include "autonomy/perception/follow/grid.hpp"
#include "autonomy/perception/follow/proto/follow.pb.h"

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

#include <string>

namespace autonomy {
namespace perception {
namespace follow {

class Planner
{
public:
    explicit Planner(proto::FollowOptions options);

    /**
     * @brief Build a clearance-checked path that stops follow_distance_m short
     *        of the target on the robot→target line.
     */
    bool Plan(const automsgs::msgs::geometry_msgs::PoseStamped& robot,
              const automsgs::msgs::geometry_msgs::PoseStamped& target,
              const LocalGrid& grid, automsgs::msgs::nav_msgs::Path* path,
              std::string* error = nullptr) const;

private:
    proto::FollowOptions options_;
};

}  // namespace follow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FOLLOW_PLANNER_HPP_
