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

#include "sensor/imu_data.hpp"

#include <cmath>
#include <cstdint>

namespace argus {
namespace sensor {
namespace {

double StampToSec(const ::automsgs::msgs::builtin_interfaces::Time& stamp) {
  return static_cast<double>(stamp.sec()) +
         1e-9 * static_cast<double>(stamp.nanosec());
}

void SetStamp(::automsgs::msgs::builtin_interfaces::Time* stamp,
              double timestamp_sec) {
  if (stamp == nullptr) {
    return;
  }
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

ImuData FromProto(const ::automsgs::msgs::sensor_msgs::Imu& msg) {
  ImuData out;
  if (msg.has_header()) {
    if (msg.header().has_stamp()) {
      out.time = StampToSec(msg.header().stamp());
    }
    out.frame_id = msg.header().frame_id();
  }
  if (msg.has_linear_acceleration()) {
    out.linear_acceleration << msg.linear_acceleration().x(),
        msg.linear_acceleration().y(), msg.linear_acceleration().z();
  }
  if (msg.has_angular_velocity()) {
    out.angular_velocity << msg.angular_velocity().x(),
        msg.angular_velocity().y(), msg.angular_velocity().z();
  }
  return out;
}

::automsgs::msgs::sensor_msgs::Imu ToProto(const ImuData& data) {
  ::automsgs::msgs::sensor_msgs::Imu msg;
  auto* header = msg.mutable_header();
  SetStamp(header->mutable_stamp(), data.time);
  header->set_frame_id(data.frame_id);
  auto* acc = msg.mutable_linear_acceleration();
  acc->set_x(data.linear_acceleration.x());
  acc->set_y(data.linear_acceleration.y());
  acc->set_z(data.linear_acceleration.z());
  auto* gyr = msg.mutable_angular_velocity();
  gyr->set_x(data.angular_velocity.x());
  gyr->set_y(data.angular_velocity.y());
  gyr->set_z(data.angular_velocity.z());
  return msg;
}

}  // namespace sensor
}  // namespace argus
