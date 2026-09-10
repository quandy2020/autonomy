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

#ifndef AUTONOMY_PERCEPTION_FOLLOW_GRID_HPP_
#define AUTONOMY_PERCEPTION_FOLLOW_GRID_HPP_

#include "autonomy/perception/follow/proto/follow.pb.h"

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/map_msgs/grid_map.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include <string>

namespace autonomy {
namespace perception {
namespace follow {

class LocalGrid
{
public:
    explicit LocalGrid(proto::FollowOptions options);

    bool Update(const automsgs::msgs::sensor_msgs::Image& depth,
                const automsgs::msgs::sensor_msgs::CameraInfo& camera,
                const automsgs::msgs::geometry_msgs::TransformStamped&
                    camera_to_map,
                const automsgs::msgs::geometry_msgs::Pose& robot_pose,
                std::string* error = nullptr);

    bool ToMessage(automsgs::msgs::map_msgs::GridMap* message) const;

    bool IsTraversable(double x, double y) const;

private:
    proto::FollowOptions options_;
    ::grid_map::GridMap map_;
};

}  // namespace follow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FOLLOW_GRID_HPP_
