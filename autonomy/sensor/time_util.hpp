/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include <chrono>

#include "autonomy/common/time.hpp"
#include <automsgs/msgs/std_msgs/header.pb.h>
#include <automsgs/msgs/time_utils.hpp>

namespace autonomy {
namespace sensor {

inline common::Time TimeFromHeader(const automsgs::msgs::std_msgs::Header & header)
{
  const auto unix_since_epoch = std::chrono::nanoseconds(
    static_cast<int64>(
      ::automsgs::msgs::builtin_interfaces::TimeToNanoseconds(header.stamp())));
  const auto uts_since_epoch = std::chrono::duration_cast<common::Duration>(
    unix_since_epoch +
    std::chrono::seconds(common::kUtsEpochOffsetFromUnixEpochInSeconds));
  return common::Time(uts_since_epoch);
}

}  // namespace sensor
}  // namespace autonomy
