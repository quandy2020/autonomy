/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_common/transformation/tf2_helpers/tf2_conversion_helpers.hpp
 * (Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file tf2_conversion_helpers.hpp
 * @brief Time / header conversions for TF helpers (no ROS / rclcpp).
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>

namespace autoviz {
namespace common {
namespace transformation {
namespace tf2_helpers {

/** Nanosecond-precision time point (tf2 @c TimePoint equivalent). */
using TimePoint =
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

automsgs::msgs::std_msgs::Header createHeader(const TimePoint& time,
                                               const std::string& frame_id);

automsgs::msgs::builtin_interfaces::Time fromTimePoint(
    const TimePoint& time);

TimePoint toTimePoint(
    const automsgs::msgs::builtin_interfaces::Time& time);

}  // namespace tf2_helpers
}  // namespace transformation
}  // namespace common
}  // namespace autoviz
