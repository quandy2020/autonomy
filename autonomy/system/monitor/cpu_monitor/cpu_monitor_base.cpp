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

#include "autonomy/system/monitor/cpu_monitor/cpu_monitor_base.hpp"

#include "autonomy/system/monitor/cpu_monitor/cpu_usage_statistics.hpp"

#include <fstream>
#include <sstream>
#include <string>

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
#include <prometheus/family.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#endif

namespace autonomy {
namespace system {
namespace monitor {
namespace cpu_monitor {

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
struct CpuMonitorBase::PrometheusGauges {
    prometheus::Family<prometheus::Gauge>* family{nullptr};
    prometheus::Gauge* total_usage{nullptr};
    std::vector<prometheus::Gauge*> per_core_usage;
};
#endif

namespace {

bool ParseCpuLine(const std::string& line, CpuTickCounts* out) {
    if (out == nullptr)
        return false;
    std::istringstream iss(line);
    std::string cpu_label;
    iss >> cpu_label;
    if (cpu_label.empty() || cpu_label.find("cpu") != 0)
        return false;
    uint64_t u, n, s, i, iow, irq, soft, steal, guest, gnice;
    if (!(iss >> u >> n >> s >> i >> iow >> irq >> soft >> steal >> guest >> gnice))
        return false;
    out->user = u;
    out->nice = n;
    out->system = s;
    out->idle = i;
    out->iowait = iow;
    out->irq = irq;
    out->softirq = soft;
    out->steal = steal;
    out->guest = guest;
    out->guest_nice = gnice;
    return true;
}

}  // namespace

void CpuMonitorBase::FillCpuInformation(CpuInformation* info) {
    if (info == nullptr)
        return;
    info->vendor = "unknown";
    info->model_name = "Unknown CPU";
    info->num_cores = 0;
    info->freq_hz = 0;
}

bool CpuMonitorBase::ReadProcStat(std::vector<CpuTickCounts>* out) {
    if (out == nullptr)
        return false;
    out->clear();
    std::ifstream f("/proc/stat");
    if (!f.is_open())
        return false;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty())
            continue;
        CpuTickCounts counts;
        if (ParseCpuLine(line, &counts))
            out->push_back(counts);
    }
    return !out->empty();
}

void CpuMonitorBase::Collect() {
    std::vector<CpuTickCounts> curr;
    if (!ReadProcStat(&curr))
        return;
    if (first_collect_) {
        prev_ticks_ = std::move(curr);
        first_collect_ = false;
        return;
    }
    if (curr.size() != prev_ticks_.size()) {
        prev_ticks_ = std::move(curr);
        return;
    }
    ComputeUsage(prev_ticks_, curr, &usage_);
    if (info_.num_cores == 0 && !curr.empty())
        info_.num_cores = static_cast<uint32_t>(curr.size());
    prev_ticks_ = std::move(curr);

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (gauges_) {
        if (gauges_->total_usage)
            gauges_->total_usage->Set(usage_.total_usage_percent);
        for (size_t i = 0; i < usage_.per_core_usage_percent.size(); ++i) {
            while (gauges_->per_core_usage.size() <= i && gauges_->family) {
                gauges_->per_core_usage.push_back(
                    &gauges_->family->Add({{"cpu", "cpu" + std::to_string(gauges_->per_core_usage.size())}}));
            }
            if (i < gauges_->per_core_usage.size() && gauges_->per_core_usage[i])
                gauges_->per_core_usage[i]->Set(usage_.per_core_usage_percent[i]);
        }
    }
#endif
}

void CpuMonitorBase::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    gauges_ = std::make_unique<PrometheusGauges>();
    auto& family = prometheus::BuildGauge()
                       .Name("autonomy_system_cpu_usage_percent")
                       .Help("CPU usage percent (total or per core)")
                       .Register(*reg);
    gauges_->family = &family;
    gauges_->total_usage = &family.Add({{"cpu", "total"}});
#else
    (void)registry;
#endif
}

std::unique_ptr<CpuMonitorBase> CreateCpuMonitor() {
    return std::make_unique<CpuMonitorBase>();
}

}  // namespace cpu_monitor
}  // namespace monitor
}  // namespace system
}  // namespace autonomy
