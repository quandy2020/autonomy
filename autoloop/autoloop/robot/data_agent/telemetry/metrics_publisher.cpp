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

#include "robot/data_agent/telemetry/metrics_publisher.hpp"
#include <fstream>
#include "common/base/json_util.hpp"

namespace autoloop {
namespace robot {

MetricsPublisher::MetricsPublisher(std::string path) : path_(std::move(path)) {}

void MetricsPublisher::Publish(const TelemetryCollector& c) {
    const auto h = c.Snapshot();
    const auto line = common::JsonObject({
        {"robot_id", h.robot_id()},
        {"dropped", std::to_string(h.dropped())},
        {"snapshots", std::to_string(h.snapshots_written())},
        {"latch", h.latch_set() ? "1" : "0"},
    });
    std::ofstream out(path_, std::ios::app);
    out << line << '\n';
}

}  // namespace robot
}  // namespace autoloop
