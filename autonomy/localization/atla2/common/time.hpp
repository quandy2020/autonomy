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

#pragma once

#include <chrono>
#include <string>

#include "autonomy/localization/atla2/common/types.hpp"

namespace autonomy::localization::atla2 {

//! Convert seconds (double) to nanoseconds TimeStamp.
inline TimeStamp SecToStamp(double sec) {
  return static_cast<TimeStamp>(sec * 1e9);
}

inline double StampToSec(TimeStamp t) {
  return static_cast<double>(t) * 1e-9;
}

inline TimeStamp NowStamp() {
  using clock = std::chrono::system_clock;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             clock::now().time_since_epoch())
      .count();
}

//! Soft sync window helper: |a - b| <= tol_ns.
inline bool WithinTol(TimeStamp a, TimeStamp b, TimeStamp tol_ns) {
  const TimeStamp d = (a > b) ? (a - b) : (b - a);
  return d <= tol_ns;
}

std::string FormatStamp(TimeStamp t);

}  // namespace autonomy::localization::atla2
