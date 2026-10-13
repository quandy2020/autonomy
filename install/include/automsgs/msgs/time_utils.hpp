/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <chrono>
#include <cstdint>

#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/builtin_interfaces/time.pb.h>

namespace automsgs {
namespace msgs {
namespace builtin_interfaces {

inline Time MakeTime(int32_t sec, uint32_t nanosec) {
  Time time;
  time.set_sec(sec);
  time.set_nanosec(nanosec);
  return time;
}

inline Time ZeroTime() { return MakeTime(0, 0); }

inline uint64_t TimeToNanoseconds(const Time& time) {
  return static_cast<uint64_t>(time.sec()) * 1'000'000'000ULL +
         static_cast<uint64_t>(time.nanosec());
}

inline Time TimeFromNanoseconds(uint64_t nanos) {
  Time time;
  time.set_sec(static_cast<int32_t>(nanos / 1'000'000'000ULL));
  time.set_nanosec(static_cast<uint32_t>(nanos % 1'000'000'000ULL));
  return time;
}

inline Time TimeNow() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto nanos =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return TimeFromNanoseconds(static_cast<uint64_t>(nanos));
}

// Compatibility aliases for former commsgs Time helpers.
inline Time Now() { return TimeNow(); }
inline uint64_t ToUnixTimeNanos(const Time& time) {
  return TimeToNanoseconds(time);
}

inline Duration DurationFromNanoseconds(int64_t nanos) {
  Duration duration;
  if (nanos >= 0) {
    duration.set_sec(static_cast<int32_t>(nanos / 1'000'000'000LL));
    duration.set_nanosec(static_cast<uint32_t>(nanos % 1'000'000'000LL));
  } else {
    const int64_t abs_nanos = -nanos;
    duration.set_sec(-static_cast<int32_t>(abs_nanos / 1'000'000'000LL));
    duration.set_nanosec(static_cast<uint32_t>(abs_nanos % 1'000'000'000LL));
    if (duration.nanosec() != 0) {
      duration.set_sec(duration.sec() - 1);
      duration.set_nanosec(1'000'000'000u - duration.nanosec());
    }
  }
  return duration;
}

inline Duration DurationFromSeconds(double seconds) {
  return DurationFromNanoseconds(
      static_cast<int64_t>(seconds * 1'000'000'000.0));
}

inline double DurationToSeconds(const Duration& duration) {
  return static_cast<double>(duration.sec()) +
         static_cast<double>(duration.nanosec()) * 1e-9;
}


inline int64_t TimeDifferenceNanoseconds(const Time& lhs, const Time& rhs) {
  return static_cast<int64_t>(TimeToNanoseconds(lhs)) -
         static_cast<int64_t>(TimeToNanoseconds(rhs));
}

inline Duration operator-(const Time& lhs, const Time& rhs) {
  return DurationFromNanoseconds(TimeDifferenceNanoseconds(lhs, rhs));
}

inline double Seconds(const Duration& duration) {
  return DurationToSeconds(duration);
}

inline bool operator>(const Duration& lhs, const Duration& rhs) {
  return DurationToSeconds(lhs) > DurationToSeconds(rhs);
}

inline bool operator<(const Duration& lhs, const Duration& rhs) {
  return DurationToSeconds(lhs) < DurationToSeconds(rhs);
}

inline bool operator>=(const Duration& lhs, const Duration& rhs) {
  return !(lhs < rhs);
}

inline bool operator<=(const Duration& lhs, const Duration& rhs) {
  return !(lhs > rhs);
}

}  // namespace builtin_interfaces
}  // namespace msgs
}  // namespace automsgs
