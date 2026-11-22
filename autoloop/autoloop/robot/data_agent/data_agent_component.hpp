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
 * @file data_agent_component.hpp
 * @brief Wires ring / trigger / snapshot / upload / latch. Independent process.
 */
#include <memory>
#include <string>
#include "common/base/status.hpp"
#include "robot/data_agent/ring_buffer/ring_buffer.hpp"
#include "robot/data_agent/safety_guard/safety_guard.hpp"
#include "robot/data_agent/snapshot_writer/snapshot_writer.hpp"
#include "robot/data_agent/telemetry/telemetry_collector.hpp"
#include "robot/data_agent/trigger_manager/trigger_manager.hpp"
#include "robot/data_agent/uploader/uploader.hpp"
#include "robot/data_agent/version_stamp/version_stamp.hpp"

namespace autoloop {
namespace robot {

class DataAgentComponent {
public:
    DataAgentComponent(std::string record_root, std::string robot_id);
    common::Status OnSample(Sample sample);
    common::Status ManualSnapshot(const std::string& reason);
    common::Status PumpUpload();
    SafetyGuard& safety() { return safety_; }
    const TelemetryCollector& telemetry() const { return tel_; }

private:
    common::Status Flush(const TriggerEvent& ev);
    std::string robot_id_;
    RingBuffer ring_;
    TriggerManager triggers_;
    SnapshotWriter writer_;
    Uploader uploader_;
    VersionStamp version_;
    SafetyGuard safety_;
    TelemetryCollector tel_;
};

}  // namespace robot
}  // namespace autoloop
