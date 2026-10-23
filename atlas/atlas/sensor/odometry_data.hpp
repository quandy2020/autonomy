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

#ifndef ATLAS_SENSOR_ODOMETRY_DATA_HPP_
#define ATLAS_SENSOR_ODOMETRY_DATA_HPP_

#include <automsgs/msgs/nav_msgs/odometry.pb.h>

#include <string>

#include "common/transform/rigid_transform.hpp"

namespace atlas {
namespace sensor {

struct OdometryData {
  double time = 0.0;
  std::string frame_id;
  std::string child_frame_id;
  transform::Rigid3d pose = transform::Rigid3d::Identity();
};

OdometryData FromProto(const ::automsgs::msgs::nav_msgs::Odometry& msg);
::automsgs::msgs::nav_msgs::Odometry ToProto(const OdometryData& data);

}  // namespace sensor
}  // namespace atlas

#endif  // ATLAS_SENSOR_ODOMETRY_DATA_HPP_
