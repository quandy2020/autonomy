/*
 * Copyright 2026 Autodriver contributors
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

/**
 * @file
 * @brief Helpers between vehicle_msgs RobotState and nav_msgs Odometry.
 */

#ifndef AUTODRIVER_CHASSIS_CONVERT_HPP_
#define AUTODRIVER_CHASSIS_CONVERT_HPP_

#include <cstdint>

#include "chassis/types.hpp"
#include <automsgs/msgs/nav_msgs/odometry.pb.h>

namespace autodriver {
namespace chassis {

inline void SetTimestampNs(::automsgs::msgs::builtin_interfaces::Time* t,
                           std::uint64_t stamp_ns) {
  if (t == nullptr) {
    return;
  }
  t->set_sec(static_cast<std::int32_t>(stamp_ns / 1'000'000'000ULL));
  t->set_nanosec(static_cast<std::uint32_t>(stamp_ns % 1'000'000'000ULL));
}

inline std::uint64_t TimestampToNs(
    const ::automsgs::msgs::builtin_interfaces::Time& t) {
  return static_cast<std::uint64_t>(t.sec()) * 1'000'000'000ULL +
         static_cast<std::uint64_t>(t.nanosec());
}

/** Derive nav_msgs/Odometry from vehicle_msgs.RobotState (same pose/twist). */
inline void RobotStateToOdometry(const ChassisState& state,
                                 const std::string& odom_frame,
                                 const std::string& base_frame,
                                 ::automsgs::msgs::nav_msgs::Odometry* out) {
  if (out == nullptr) {
    return;
  }
  out->Clear();
  auto* header = out->mutable_header();
  *header->mutable_stamp() = state.timestamp();
  header->set_frame_id(odom_frame.empty() ? state.global_frame() : odom_frame);
  if (header->frame_id().empty()) {
    header->set_frame_id("odom");
  }
  out->set_child_frame_id(base_frame.empty() ? "base_link" : base_frame);

  if (state.has_pose() && state.pose().has_pose()) {
    *out->mutable_pose()->mutable_pose() = state.pose().pose();
  }
  if (state.has_twist() && state.twist().has_twist()) {
    *out->mutable_twist()->mutable_twist() = state.twist().twist();
  }
}

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_CONVERT_HPP_
