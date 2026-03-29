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

#include <cstdint>
#include <memory>
#include <string>

#include "autonomy/system/monitor/monitor_base.hpp"

namespace autonomy {
namespace system {
namespace monitor {

class MemMonitor : public MonitorBase
{
public:
    std::string Name() const override {
        return "mem";
    }
    void Collect() override;
    void RegisterWithPrometheus(void* registry) override;

    uint64_t total_kb() const {
        return total_kb_;
    }
    uint64_t available_kb() const {
        return available_kb_;
    }
    double usage_percent() const {
        return usage_percent_;
    }

    static std::unique_ptr<MemMonitor> Create() {
        return std::make_unique<MemMonitor>();
    }

private:
    uint64_t total_kb_{0};
    uint64_t available_kb_{0};
    double usage_percent_{0.0};

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    void* usage_gauge_{nullptr};
    void* total_kb_gauge_{nullptr};
    void* available_kb_gauge_{nullptr};
#endif
};

inline std::unique_ptr<MemMonitor> CreateMemMonitor() {
    return MemMonitor::Create();
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
