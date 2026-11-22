/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file time_util.hpp
 * @brief Dual-clock wrappers. Prefer autolink::Time when linked.
 */

#include <cstdint>

#include "common/proto/version_tuple.pb.h"

namespace autoloop {
namespace common {

/**
 * @brief Read monotonic + UTC nanoseconds via autolink::Time when available.
 * @return DualClock proto.
 */
DualClock NowDualClock();

/**
 * @brief Sleep for @p ns nanoseconds (autolink::Duration when available).
 * @param [in] ns: Duration.
 */
void SleepNs(int64_t ns);

}  // namespace common
}  // namespace autoloop
