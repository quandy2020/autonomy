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

#include "sensor/fixed_frame_pose_data.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>

namespace atlas {
namespace sensor {
namespace {

double StampToSec(const ::automsgs::msgs::builtin_interfaces::Time& stamp) {
  return static_cast<double>(stamp.sec()) +
         1e-9 * static_cast<double>(stamp.nanosec());
}

void SetStamp(::automsgs::msgs::builtin_interfaces::Time* stamp,
              double timestamp_sec) {
  auto sec = static_cast<int32_t>(timestamp_sec);
  auto nanosec = static_cast<uint32_t>(
      std::llround((timestamp_sec - static_cast<double>(sec)) * 1e9));
  if (nanosec >= 1000000000u) {
    stamp->set_sec(sec + 1);
    stamp->set_nanosec(nanosec - 1000000000u);
  } else {
    stamp->set_sec(sec);
    stamp->set_nanosec(nanosec);
  }
}

}  // namespace

FixedFramePoseData FromProto(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& msg) {
  FixedFramePoseData data;
  if (msg.has_header() && msg.header().has_stamp()) {
    data.time = StampToSec(msg.header().stamp());
    data.frame_id = msg.header().frame_id();
  }
  if (msg.has_pose()) {
    const auto& pose = msg.pose();
    data.pose = transform::Rigid3d(
        Eigen::Vector3d(pose.position().x(), pose.position().y(),
                        pose.position().z()),
        Eigen::Quaterniond(pose.orientation().w(), pose.orientation().x(),
                           pose.orientation().y(), pose.orientation().z()));
  }
  return data;
}

::automsgs::msgs::geometry_msgs::PoseStamped ToProto(
    const FixedFramePoseData& data) {
  ::automsgs::msgs::geometry_msgs::PoseStamped msg;
  SetStamp(msg.mutable_header()->mutable_stamp(), data.time);
  msg.mutable_header()->set_frame_id(data.frame_id);
  if (data.pose.has_value()) {
    msg.mutable_pose()->mutable_position()->set_x(data.pose->translation().x());
    msg.mutable_pose()->mutable_position()->set_y(data.pose->translation().y());
    msg.mutable_pose()->mutable_position()->set_z(data.pose->translation().z());
    msg.mutable_pose()->mutable_orientation()->set_w(data.pose->rotation().w());
    msg.mutable_pose()->mutable_orientation()->set_x(data.pose->rotation().x());
    msg.mutable_pose()->mutable_orientation()->set_y(data.pose->rotation().y());
    msg.mutable_pose()->mutable_orientation()->set_z(data.pose->rotation().z());
  }
  return msg;
}

}  // namespace sensor
}  // namespace atlas
