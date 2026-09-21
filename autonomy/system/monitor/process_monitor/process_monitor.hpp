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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/system/monitor/monitor_base.hpp"
#include "autonomy/system/monitor/ops_types.hpp"

namespace autonomy {
namespace system {
namespace monitor {

class ProcessMonitor : public MonitorBase
{
public:
    std::string Name() const override {
        return "process";
    }
    void Collect() override;
    void RegisterWithPrometheus(void* registry) override;

    void set_critical_specs(std::vector<CriticalProcessSpec> specs) {
        critical_specs_ = std::move(specs);
    }
    void set_restart_hints(std::vector<std::string> hints) {
        restart_hints_ = std::move(hints);
    }

    uint32_t process_count() const {
        return process_count_;
    }
    uint64_t total_rss_kb() const {
        return total_rss_kb_;
    }
    const std::string& top_rss_comm() const {
        return top_rss_comm_;
    }
    const std::vector<ProcessHealth>& critical_health() const {
        return critical_health_;
    }

    static std::unique_ptr<ProcessMonitor> Create() {
        return std::make_unique<ProcessMonitor>();
    }

private:
    uint32_t process_count_{0};
    uint64_t total_rss_kb_{0};
    std::string top_rss_comm_;
    std::vector<CriticalProcessSpec> critical_specs_;
    std::vector<std::string> restart_hints_;
    std::vector<ProcessHealth> critical_health_;

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    void* count_gauge_{nullptr};
    void* rss_gauge_{nullptr};
#endif
};

inline std::unique_ptr<ProcessMonitor> CreateProcessMonitor() {
    return ProcessMonitor::Create();
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
