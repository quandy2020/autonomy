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

#include "autonomy/system/monitor/mem_monitor/mem_monitor.hpp"

#include <fstream>
#include <sstream>
#include <string>

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#endif

namespace autonomy {
namespace system {
namespace monitor {

namespace {

bool ParseMemInfo(uint64_t* total_kb, uint64_t* available_kb,
                  uint64_t* swap_total_kb, uint64_t* swap_free_kb) {
    std::ifstream f("/proc/meminfo");
    if (!f.is_open())
        return false;
    uint64_t mem_total = 0, mem_available = 0;
    uint64_t swap_total = 0, swap_free = 0;
    std::string line;
    while (std::getline(f, line)) {
        if (line.find("MemTotal:") == 0) {
            std::istringstream iss(line.substr(9));
            iss >> mem_total;
        } else if (line.find("MemAvailable:") == 0) {
            std::istringstream iss(line.substr(13));
            iss >> mem_available;
        } else if (line.find("SwapTotal:") == 0) {
            std::istringstream iss(line.substr(10));
            iss >> swap_total;
        } else if (line.find("SwapFree:") == 0) {
            std::istringstream iss(line.substr(9));
            iss >> swap_free;
        }
    }
    if (total_kb)
        *total_kb = mem_total;
    if (available_kb)
        *available_kb = mem_available;
    if (swap_total_kb)
        *swap_total_kb = swap_total;
    if (swap_free_kb)
        *swap_free_kb = swap_free;
    return mem_total > 0;
}

}  // namespace

void MemMonitor::Collect() {
    uint64_t total = 0, avail = 0, swap_total = 0, swap_free = 0;
    if (ParseMemInfo(&total, &avail, &swap_total, &swap_free)) {
        total_kb_ = total;
        available_kb_ = avail;
        swap_total_kb_ = swap_total;
        swap_free_kb_ = swap_free;
        usage_percent_ =
            (total > 0 && total >= avail)
                ? (100.0 * (total - avail) / static_cast<double>(total))
                : 0.0;
    }
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (usage_gauge_)
        static_cast<prometheus::Gauge*>(usage_gauge_)->Set(usage_percent_);
    if (total_kb_gauge_)
        static_cast<prometheus::Gauge*>(total_kb_gauge_)
            ->Set(static_cast<double>(total_kb_));
    if (available_kb_gauge_)
        static_cast<prometheus::Gauge*>(available_kb_gauge_)
            ->Set(static_cast<double>(available_kb_));
    if (swap_used_kb_gauge_ && swap_total_kb_ >= swap_free_kb_)
        static_cast<prometheus::Gauge*>(swap_used_kb_gauge_)
            ->Set(static_cast<double>(swap_total_kb_ - swap_free_kb_));
#endif
}

void MemMonitor::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    auto& usage_family = prometheus::BuildGauge()
                             .Name("autonomy_system_mem_usage_percent")
                             .Help("Memory usage percent")
                             .Register(*reg);
    usage_gauge_ = &usage_family.Add({});
    auto& total_family = prometheus::BuildGauge()
                             .Name("autonomy_system_mem_total_kb")
                             .Help("Total memory in KB")
                             .Register(*reg);
    total_kb_gauge_ = &total_family.Add({});
    auto& avail_family = prometheus::BuildGauge()
                             .Name("autonomy_system_mem_available_kb")
                             .Help("Available memory in KB")
                             .Register(*reg);
    available_kb_gauge_ = &avail_family.Add({});
    auto& swap_family = prometheus::BuildGauge()
                            .Name("autonomy_system_mem_swap_used_kb")
                            .Help("Swap used in KB")
                            .Register(*reg);
    swap_used_kb_gauge_ = &swap_family.Add({});
#else
    (void)registry;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
