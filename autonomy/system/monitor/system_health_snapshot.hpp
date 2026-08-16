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

#include <string>
#include <vector>

/**
 * Mapping between autonomy/system/monitor runtime types and
 * automsgs.rpcs.system SystemHealth (see automsgs/proto/rpcs/system.proto).
 *
 * | C++ (monitor)              | RPC field                                      |
 * |----------------------------|------------------------------------------------|
 * | HazardLevel::kOk/Warn/Error| HazardLevel OK/WARN/ERROR                      |
 * | MrmHandler::active()       | SystemHealth.mrm_active                        |
 * | ChannelHealth              | ChannelHealth {channel, healthy, age, rate}    |
 * | LatencyHealth              | LatencyHealth {channel, message_age_seconds}   |
 * | CPU/Mem/HDD/NTP gauges     | HostResourceSnapshot                           |
 *
 * SystemService.GetStatus embeds a compact SystemHealth;
 * SystemService.GetHealth returns the full snapshot for diagnostics.
 */

#include "autonomy/system/monitor/ops_types.hpp"

namespace autonomy {
namespace system {
namespace monitor {

/// Documented alias: fill from MonitorRegistry collectors when bridging to gRPC.
struct SystemHealthSnapshot {
    HazardLevel hazard_level{HazardLevel::kOk};
    bool mrm_active{false};
    bool emergency_stop_latched{false};
    float cpu_usage_percent{-1.f};
    float memory_usage_percent{-1.f};
    float disk_usage_percent{-1.f};
    float load_average_1m{-1.f};
    float ntp_offset_seconds{0.f};
    std::vector<ChannelHealth> channels;
    std::vector<LatencyHealth> latencies;
    std::string detail;
};

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
