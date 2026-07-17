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

#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "autonomy/system/monitor/cpu_monitor/cpu_usage_statistics.hpp"
#include "autonomy/system/monitor/system_monitor_utility.hpp"

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
    prometheus::Family<prometheus::Gauge>* usage_family{nullptr};
    prometheus::Gauge* total_usage{nullptr};
    std::vector<prometheus::Gauge*> per_core_usage;
    prometheus::Family<prometheus::Gauge>* temp_family{nullptr};
    std::vector<prometheus::Gauge*> temperature;
    prometheus::Family<prometheus::Gauge>* load_family{nullptr};
    prometheus::Gauge* load_1{nullptr};
    prometheus::Gauge* load_5{nullptr};
    prometheus::Gauge* load_15{nullptr};
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
    if (!(iss >> u >> n >> s >> i >> iow >> irq >> soft >> steal >> guest >>
          gnice))
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

bool ReadMillidegreeCelsius(const std::string& path, double* celsius) {
    if (celsius == nullptr)
        return false;
    std::ifstream f(path);
    if (!f.is_open())
        return false;
    int64_t millideg = 0;
    if (!(f >> millideg))
        return false;
    *celsius = static_cast<double>(millideg) / 1000.0;
    return true;
}

void TrimInPlace(std::string* s) {
    if (s == nullptr || s->empty())
        return;
    while (!s->empty() && (s->back() == ' ' || s->back() == '\t'))
        s->pop_back();
    size_t start = 0;
    while (start < s->size() && ((*s)[start] == ' ' || (*s)[start] == '\t'))
        ++start;
    if (start > 0)
        s->erase(0, start);
}

}  // namespace

void CpuMonitorBase::FillCpuInformation(CpuInformation* info) {
    if (info == nullptr)
        return;
    info->vendor = "unknown";
    info->model_name = "Unknown CPU";
    info->num_cores = 0;
    info->freq_hz = 0;

    std::ifstream f("/proc/cpuinfo");
    if (!f.is_open())
        return;

    uint32_t logical_cpus = 0;
    double mhz = 0.0;
    std::string line;
    while (std::getline(f, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos)
            continue;
        std::string key = line.substr(0, colon);
        std::string value = line.substr(colon + 1);
        TrimInPlace(&key);
        TrimInPlace(&value);
        if (key == "vendor_id") {
            if (value.find("GenuineIntel") != std::string::npos)
                info->vendor = "intel";
            else if (value.find("AuthenticAMD") != std::string::npos)
                info->vendor = "amd";
            else if (value.find("ARM") != std::string::npos ||
                     value.find("aarch") != std::string::npos)
                info->vendor = "arm";
        } else if (key == "model name" && info->model_name == "Unknown CPU") {
            info->model_name = value;
        } else if (key == "processor") {
            ++logical_cpus;
        } else if (key == "cpu MHz" && mhz == 0.0) {
            try {
                mhz = std::stod(value);
            } catch (...) {
            }
        }
    }
    if (logical_cpus == 0)
        logical_cpus = std::thread::hardware_concurrency();
    info->num_cores = logical_cpus;
    if (mhz > 0.0)
        info->freq_hz = static_cast<uint64_t>(mhz * 1e6);
}

void CpuMonitorBase::EnsureCpuInformation() {
    if (cpu_info_loaded_)
        return;
    FillCpuInformation(&info_);
    cpu_info_loaded_ = true;
}

void CpuMonitorBase::EnsureTemperatureSensorPaths() {
    if (temperature_paths_initialized_)
        return;

    std::vector<ThermalZone> zones;
    SystemMonitorUtility::GetIntelCoretempInputs(&zones);
    static const char* kThermalPrefixes[] = {
        "cpu-therm", "x86_pkg_temp", "acpitz", "soc-thermal",
    };
    for (const char* prefix : kThermalPrefixes)
        SystemMonitorUtility::GetThermalZonesByTypePrefix(prefix, &zones);

    for (const auto& z : zones) {
        if (z.temperature_path.empty())
            continue;
        temperature_sensor_paths_.emplace_back(z.label, z.temperature_path);
    }
    temperature_paths_initialized_ = true;
}

void CpuMonitorBase::ReadTemperatures() {
    EnsureTemperatureSensorPaths();
    temperatures_.clear();
    temperatures_.reserve(temperature_sensor_paths_.size());
    for (const auto& [label, path] : temperature_sensor_paths_) {
        CpuTemperatureReading reading;
        reading.label = label;
        if (ReadMillidegreeCelsius(path, &reading.celsius))
            temperatures_.push_back(reading);
    }
}

void CpuMonitorBase::ReadLoadAverage() {
    std::ifstream f("/proc/loadavg");
    if (!f.is_open())
        return;
    f >> load_average_.load_1 >> load_average_.load_5 >> load_average_.load_15;
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
        if (line.rfind("cpu", 0) != 0)
            break;
        CpuTickCounts counts;
        if (ParseCpuLine(line, &counts))
            out->push_back(counts);
    }
    return !out->empty();
}

void CpuMonitorBase::Collect() {
    EnsureCpuInformation();
    ReadLoadAverage();
    ReadTemperatures();

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
    if (info_.num_cores == 0 && curr.size() > 1)
        info_.num_cores = static_cast<uint32_t>(curr.size() - 1);
    prev_ticks_ = std::move(curr);

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (gauges_) {
        if (gauges_->total_usage)
            gauges_->total_usage->Set(usage_.total_usage_percent);
        for (size_t i = 0; i < usage_.per_core_usage_percent.size(); ++i) {
            while (gauges_->per_core_usage.size() <= i && gauges_->usage_family) {
                gauges_->per_core_usage.push_back(&gauges_->usage_family->Add(
                    {{"cpu", "cpu" + std::to_string(
                                         gauges_->per_core_usage.size())}}));
            }
            if (i < gauges_->per_core_usage.size() &&
                gauges_->per_core_usage[i])
                gauges_->per_core_usage[i]->Set(
                    usage_.per_core_usage_percent[i]);
        }
        if (gauges_->temp_family) {
            for (size_t i = 0; i < temperatures_.size(); ++i) {
                while (gauges_->temperature.size() <= i) {
                    const auto& label = temperatures_[i].label;
                    gauges_->temperature.push_back(&gauges_->temp_family->Add(
                        {{"sensor", label.empty() ? ("zone" + std::to_string(i))
                                                    : label}}));
                }
                if (i < gauges_->temperature.size() && gauges_->temperature[i])
                    gauges_->temperature[i]->Set(temperatures_[i].celsius);
            }
        }
        if (gauges_->load_1)
            gauges_->load_1->Set(load_average_.load_1);
        if (gauges_->load_5)
            gauges_->load_5->Set(load_average_.load_5);
        if (gauges_->load_15)
            gauges_->load_15->Set(load_average_.load_15);
    }
#endif
}

void CpuMonitorBase::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    gauges_ = std::make_unique<PrometheusGauges>();
    auto& usage_family = prometheus::BuildGauge()
                             .Name("autonomy_system_cpu_usage_percent")
                             .Help("CPU usage percent (total or per core)")
                             .Register(*reg);
    gauges_->usage_family = &usage_family;
    gauges_->total_usage = &usage_family.Add({{"cpu", "total"}});

    auto& temp_family = prometheus::BuildGauge()
                            .Name("autonomy_system_cpu_temperature_celsius")
                            .Help("CPU thermal sensor reading in Celsius")
                            .Register(*reg);
    gauges_->temp_family = &temp_family;

    auto& load_family = prometheus::BuildGauge()
                            .Name("autonomy_system_load_average")
                            .Help("Load average from /proc/loadavg")
                            .Register(*reg);
    gauges_->load_family = &load_family;
    gauges_->load_1 = &load_family.Add({{"period", "1m"}});
    gauges_->load_5 = &load_family.Add({{"period", "5m"}});
    gauges_->load_15 = &load_family.Add({{"period", "15m"}});
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
