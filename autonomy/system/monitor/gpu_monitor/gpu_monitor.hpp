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

#include <memory>
#include <string>
#include <vector>

#include "autonomy/system/monitor/monitor_base.hpp"

namespace autonomy {
namespace system {
namespace monitor {

struct GpuBusyReading {
    std::string card;
    double busy_percent{0.0};
};

class GpuMonitor : public MonitorBase
{
public:
    std::string Name() const override {
        return "gpu";
    }
    void Collect() override;
    void RegisterWithPrometheus(void* registry) override;

    const std::vector<GpuBusyReading>& gpus() const {
        return gpus_;
    }

    static std::unique_ptr<GpuMonitor> Create() {
        return std::make_unique<GpuMonitor>();
    }

private:
    std::vector<GpuBusyReading> gpus_;

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    struct PrometheusGauges;
    std::unique_ptr<PrometheusGauges> gauges_;
#endif
};

inline std::unique_ptr<GpuMonitor> CreateGpuMonitor() {
    return GpuMonitor::Create();
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
