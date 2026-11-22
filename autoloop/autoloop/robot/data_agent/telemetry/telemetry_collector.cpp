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

#include "robot/data_agent/telemetry/telemetry_collector.hpp"
#include "common/base/time_util.hpp"

namespace autoloop {
namespace robot {

void TelemetryCollector::SetRobotId(std::string id) {
    std::lock_guard<std::mutex> lock(mu_);
    h_.set_robot_id(std::move(id));
}
void TelemetryCollector::AddDropped(uint64_t n) {
    std::lock_guard<std::mutex> lock(mu_);
    h_.set_dropped(h_.dropped() + n);
}
void TelemetryCollector::SetLatch(bool on) {
    std::lock_guard<std::mutex> lock(mu_);
    h_.set_latch_set(on);
}
void TelemetryCollector::SetRingSamples(uint64_t n) {
    std::lock_guard<std::mutex> lock(mu_);
    h_.set_ring_samples(n);
}
void TelemetryCollector::SetSnapshots(uint64_t n) {
    std::lock_guard<std::mutex> lock(mu_);
    h_.set_snapshots_written(n);
}
void TelemetryCollector::SetUploadPending(uint64_t n) {
    std::lock_guard<std::mutex> lock(mu_);
    h_.set_upload_pending(n);
}
void TelemetryCollector::SetError(std::string e) {
    std::lock_guard<std::mutex> lock(mu_);
    h_.set_last_error(std::move(e));
}
common::DataAgentHealth TelemetryCollector::Snapshot() const {
    std::lock_guard<std::mutex> lock(mu_);
    auto o = h_;
    o.set_last_heartbeat_utc_ns(common::NowDualClock().utc_ns());
    return o;
}

}  // namespace robot
}  // namespace autoloop
