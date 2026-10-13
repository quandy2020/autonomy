/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_common/transformation/tf2_helpers/tf2_conversion_helpers.cpp
 * (Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/common/transformation/tf2_helpers/tf2_conversion_helpers.hpp"

#include <chrono>
#include <cstdint>

namespace autoviz {
namespace common {
namespace transformation {
namespace tf2_helpers {

automsgs::msgs::std_msgs::Header createHeader(const TimePoint& time,
                                              const std::string& frame_id) {
  automsgs::msgs::std_msgs::Header header;
  const auto seconds =
      std::chrono::time_point_cast<std::chrono::seconds>(time);
  const auto nanoseconds =
      std::chrono::time_point_cast<std::chrono::nanoseconds>(time) -
      std::chrono::time_point_cast<std::chrono::nanoseconds>(seconds);
  header.mutable_stamp()->set_sec(
      static_cast<int32_t>(seconds.time_since_epoch().count()));
  header.mutable_stamp()->set_nanosec(
      static_cast<uint32_t>(nanoseconds.count()));
  header.set_frame_id(frame_id);
  return header;
}

automsgs::msgs::builtin_interfaces::Time fromTimePoint(
    const TimePoint& time) {
  automsgs::msgs::builtin_interfaces::Time stamp;
  const auto seconds =
      std::chrono::time_point_cast<std::chrono::seconds>(time);
  const auto nanoseconds =
      std::chrono::time_point_cast<std::chrono::nanoseconds>(time) -
      std::chrono::time_point_cast<std::chrono::nanoseconds>(seconds);
  stamp.set_sec(static_cast<int32_t>(seconds.time_since_epoch().count()));
  stamp.set_nanosec(static_cast<uint32_t>(nanoseconds.count()));
  return stamp;
}

TimePoint toTimePoint(
    const automsgs::msgs::builtin_interfaces::Time& time) {
  return TimePoint{std::chrono::seconds{time.sec()} +
                   std::chrono::nanoseconds{time.nanosec()}};
}

}  // namespace tf2_helpers
}  // namespace transformation
}  // namespace common
}  // namespace autoviz
