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
#include <map>
#include <memory>
#include <string>

#include "autonomy/system/monitor/monitor_base.hpp"

namespace autonomy {
namespace system {
namespace monitor {

struct NetInterfaceStats {
    std::string name;
    uint64_t rx_bytes{0};
    uint64_t tx_bytes{0};
    double rx_bytes_per_sec{0.0};
    double tx_bytes_per_sec{0.0};
};

class NetMonitor : public MonitorBase
{
public:
    std::string Name() const override {
        return "net";
    }
    void Collect() override;
    void RegisterWithPrometheus(void* registry) override;

    const std::map<std::string, NetInterfaceStats>& interfaces() const {
        return interfaces_;
    }

    static std::unique_ptr<NetMonitor> Create() {
        return std::make_unique<NetMonitor>();
    }

private:
    std::map<std::string, NetInterfaceStats> interfaces_;
    std::map<std::string, std::pair<uint64_t, uint64_t>> prev_bytes_;
    std::chrono::steady_clock::time_point prev_time_{};
    bool first_collect_{true};

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    struct PrometheusGauges;
    std::unique_ptr<PrometheusGauges> gauges_;
#endif
};

inline std::unique_ptr<NetMonitor> CreateNetMonitor() {
    return NetMonitor::Create();
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
