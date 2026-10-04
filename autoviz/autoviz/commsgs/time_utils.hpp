/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file time_utils.hpp
 * @brief Automsgs @c builtin_interfaces/Time helpers for Autoviz.
 *
 * Provides a zero timestamp and nanosecond conversion used by TF lookups and
 * message stamp comparisons.
 *
 * @see transform::Buffer::lookupTransform
 */

#pragma once

#include <automsgs/msgs/builtin_interfaces/time.pb.h>

namespace autoviz {
namespace commsgs {

/**
 * @brief Returns a default-constructed (zero) Automsgs time.
 *
 * Equivalent to “latest / unset” semantics for many TF query paths when both
 * @c sec and @c nanosec are zero.
 *
 * @return Zero @c builtin_interfaces::Time.
 */
inline automsgs::msgs::builtin_interfaces::Time ZeroTime() {
  return {};
}

/**
 * @brief Converts Automsgs time to a single nanosecond count since epoch.
 *
 * @param time Stamp with @c sec and @c nanosec fields.
 * @return @c sec * 1e9 + @c nanosec as @c uint64_t.
 *
 * @note Does not check for overflow on extreme @c sec values.
 */
inline uint64_t TimeToNanoseconds(
    const automsgs::msgs::builtin_interfaces::Time& time) {
  return static_cast<uint64_t>(time.sec()) * 1'000'000'000ULL +
         static_cast<uint64_t>(time.nanosec());
}

}  // namespace commsgs
}  // namespace autoviz
