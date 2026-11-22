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

#include "robot/data_agent/structured_log/custom_prefix.hpp"

#include <sstream>

#include "common/base/time_util.hpp"

namespace autoloop {
namespace robot {

std::string FormatLogPrefix(const std::string& trace_id) {
    const auto c = common::NowDualClock();
    std::ostringstream ss;
    ss << '[' << c.utc_ns() << "][" << c.mono_ns() << "][" << trace_id << ']';
    return ss.str();
}

}  // namespace robot
}  // namespace autoloop
