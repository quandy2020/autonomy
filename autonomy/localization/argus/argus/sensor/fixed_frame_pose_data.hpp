/*
 * Copyright 2026 The Openbot Authors
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

#ifndef ARGUS_SENSOR_FIXED_FRAME_POSE_DATA_HPP_
#define ARGUS_SENSOR_FIXED_FRAME_POSE_DATA_HPP_

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>

#include <optional>
#include <string>

#include "common/transform/rigid_transform.hpp"

namespace argus {
namespace sensor {

struct FixedFramePoseData {
  double time = 0.0;
  std::string frame_id;
  std::optional<transform::Rigid3d> pose;
};

FixedFramePoseData FromProto(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& msg);
::automsgs::msgs::geometry_msgs::PoseStamped ToProto(
    const FixedFramePoseData& data);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_FIXED_FRAME_POSE_DATA_HPP_
