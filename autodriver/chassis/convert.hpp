/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file convert.hpp
 * @brief Helpers between vehicle_msgs RobotState and nav_msgs Odometry.
 */

#ifndef AUTODRIVER_CHASSIS_CONVERT_HPP_
#define AUTODRIVER_CHASSIS_CONVERT_HPP_

#include <cstdint>
#include <string>

#include "chassis/types.hpp"
#include <automsgs/msgs/nav_msgs/odometry.pb.h>

namespace autodriver {
namespace chassis {

/**
 * @brief Write sec/nanosec into a builtin_interfaces Time from epoch nanoseconds.
 * @param[out] t Destination timestamp; no-op when null.
 * @param[in] stamp_ns Epoch time in nanoseconds.
 */
inline void FillTimestampFromNanoseconds(
    ::automsgs::msgs::builtin_interfaces::Time* t, std::uint64_t stamp_ns) {
  if (t == nullptr) {
    return;
  }
  t->set_sec(static_cast<std::int32_t>(stamp_ns / 1'000'000'000ULL));
  t->set_nanosec(static_cast<std::uint32_t>(stamp_ns % 1'000'000'000ULL));
}

/**
 * @brief Convert builtin_interfaces Time to epoch nanoseconds.
 * @param[in] t Source timestamp (sec + nanosec).
 * @return Epoch time in nanoseconds.
 */
inline std::uint64_t ConvertTimestampToNanoseconds(
    const ::automsgs::msgs::builtin_interfaces::Time& t) {
  return static_cast<std::uint64_t>(t.sec()) * 1'000'000'000ULL +
         static_cast<std::uint64_t>(t.nanosec());
}

/**
 * @brief Convert vehicle_msgs.RobotState pose/twist into nav_msgs/Odometry.
 * @param[in] state Chassis RobotState sample (pose / twist / timestamp).
 * @param[in] odom_frame Header frame_id; falls back to state.global_frame / "odom".
 * @param[in] base_frame child_frame_id; defaults to "base_link" when empty.
 * @param[out] out Output odometry message (cleared then filled); no-op if null.
 */
inline void ConvertRobotStateToOdometry(
    const ChassisState& state, const std::string& odom_frame,
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

  if (state.has_pose()) {
    // PoseWithCovariance.pose is PoseStamped in automsgs.
    *out->mutable_pose()->mutable_pose() = state.pose();
  }
  if (state.has_twist() && state.twist().has_twist()) {
    *out->mutable_twist()->mutable_twist() = state.twist().twist();
  }
}

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_CONVERT_HPP_
