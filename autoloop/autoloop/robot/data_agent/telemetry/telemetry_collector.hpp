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
 * @file telemetry_collector.hpp
 * @brief In-process counters for data_agent_health.
 */
#include <cstdint>
#include <mutex>
#include "common/proto/data_agent_health.pb.h"

namespace autoloop {
namespace robot {

class TelemetryCollector {
public:
    void SetRobotId(std::string id);
    void AddDropped(uint64_t n);
    void SetLatch(bool on);
    void SetRingSamples(uint64_t n);
    void SetSnapshots(uint64_t n);
    void SetUploadPending(uint64_t n);
    void SetError(std::string e);
    common::DataAgentHealth Snapshot() const;

private:
    mutable std::mutex mu_;
    common::DataAgentHealth h_;
};

}  // namespace robot
}  // namespace autoloop
