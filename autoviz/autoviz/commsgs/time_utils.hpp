/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <automsgs/msgs/builtin_interfaces/time.pb.h>

namespace autoviz {
namespace commsgs {

inline automsgs::msgs::builtin_interfaces::Time ZeroTime() {
  return {};
}

inline uint64_t TimeToNanoseconds(
    const automsgs::msgs::builtin_interfaces::Time& time) {
  return static_cast<uint64_t>(time.sec()) * 1'000'000'000ULL +
         static_cast<uint64_t>(time.nanosec());
}

}  // namespace commsgs
}  // namespace autoviz
