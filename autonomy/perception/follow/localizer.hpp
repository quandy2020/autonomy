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

#ifndef AUTONOMY_PERCEPTION_FOLLOW_LOCALIZER_HPP_
#define AUTONOMY_PERCEPTION_FOLLOW_LOCALIZER_HPP_

#include "autonomy/perception/follow/proto/follow.pb.h"

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d.pb.h>

#include <string>

namespace autonomy {
namespace perception {
namespace follow {

class Localizer
{
public:
    explicit Localizer(proto::FollowOptions options);

    /**
     * @brief Lift a tracked person box with depth into map-frame PoseStamped.
     */
    bool Localize(const automsgs::msgs::vision_msgs::Detection2D& track,
                  const automsgs::msgs::sensor_msgs::Image& depth,
                  const automsgs::msgs::sensor_msgs::CameraInfo& camera,
                  const automsgs::msgs::geometry_msgs::TransformStamped&
                      camera_to_map,
                  automsgs::msgs::geometry_msgs::PoseStamped* target,
                  std::string* error = nullptr) const;

private:
    proto::FollowOptions options_;
};

}  // namespace follow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FOLLOW_LOCALIZER_HPP_
