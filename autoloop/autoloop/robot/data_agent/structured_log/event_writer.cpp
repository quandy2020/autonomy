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

#include "robot/data_agent/structured_log/event_writer.hpp"

#include "common/base/json_util.hpp"
#include "common/base/time_util.hpp"
#include "robot/data_agent/structured_log/trace_context.hpp"

namespace autoloop {
namespace robot {

EventWriter::EventWriter(std::string path) : path_(std::move(path)) {}

void EventWriter::Write(EventCode code, const std::string& payload_json) {
    const auto c = common::NowDualClock();
    const auto line = common::JsonObject({
        {"ts_mono_ns", std::to_string(c.mono_ns())},
        {"ts_utc_ns", std::to_string(c.utc_ns())},
        {"trace_id", CurrentTraceId()},
        {"event_code", EventCodeName(code)},
        {"payload", payload_json},
    });
    std::lock_guard<std::mutex> lock(mu_);
    std::ofstream out(path_, std::ios::app);
    out << line << '\n';
}

}  // namespace robot
}  // namespace autoloop
