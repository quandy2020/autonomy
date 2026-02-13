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

#include "autonomy/system/monitor/monitor_registry.hpp"

#include "autonomy/system/monitor/cpu_monitor/cpu_monitor_base.hpp"
#include "autonomy/system/monitor/gpu_monitor/gpu_monitor.hpp"
#include "autonomy/system/monitor/hdd_monitor/hdd_monitor.hpp"
#include "autonomy/system/monitor/mem_monitor/mem_monitor.hpp"
#include "autonomy/system/monitor/net_monitor/net_monitor.hpp"
#include "autonomy/system/monitor/ntp_monitor/ntp_monitor.hpp"
#include "autonomy/system/monitor/process_monitor/process_monitor.hpp"
#include "autonomy/system/monitor/voltage_monitor/voltage_monitor.hpp"

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
#include <prometheus/exposer.h>
#include <prometheus/registry.h>
#endif

namespace autonomy {
namespace system {
namespace monitor {

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
struct MonitorRegistry::PrometheusState {
    std::shared_ptr<prometheus::Registry> registry;
    std::unique_ptr<prometheus::Exposer> exposer;
};
#endif

MonitorRegistry::MonitorRegistry(MonitorOptions options) : options_(std::move(options)) {}

MonitorRegistry::~MonitorRegistry() {
    Stop();
}

void MonitorRegistry::Start() {
    if (started_)
        return;
    BuildMonitorsFromOptions();

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (options_.enable_prometheus && !options_.prometheus_bind_address.empty()) {
        prometheus_ = std::make_unique<PrometheusState>();
        prometheus_->registry = std::make_shared<prometheus::Registry>();
        prometheus_->exposer = std::make_unique<prometheus::Exposer>(options_.prometheus_bind_address);
        prometheus_->exposer->RegisterCollectable(prometheus_->registry);
        for (auto& m : monitors_) {
            if (m && m->enabled())
                m->RegisterWithPrometheus(prometheus_->registry.get());
        }
    }
#endif

    StartGperfIfRequested();
    started_ = true;
}

void MonitorRegistry::Stop() {
    if (!started_)
        return;
    StopGperfIfActive();
    started_ = false;
}

void MonitorRegistry::CollectAll() {
    for (auto& m : monitors_) {
        if (m && m->enabled())
            m->Collect();
    }
}

void MonitorRegistry::AddMonitor(std::unique_ptr<MonitorBase> monitor) {
    if (!monitor)
        return;
    monitors_.push_back(std::move(monitor));
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (prometheus_ && prometheus_->registry && monitors_.back()->enabled())
        monitors_.back()->RegisterWithPrometheus(prometheus_->registry.get());
#endif
}

void MonitorRegistry::BuildMonitorsFromOptions() {
    using namespace cpu_monitor;
    if (options_.enable_cpu_monitor) {
        auto cpu = CreateCpuMonitor();
        cpu->set_enabled(true);
        monitors_.push_back(std::move(cpu));
    }
    if (options_.enable_gpu_monitor) {
        auto gpu = CreateGpuMonitor();
        gpu->set_enabled(true);
        monitors_.push_back(std::move(gpu));
    }
    if (options_.enable_mem_monitor) {
        auto mem = CreateMemMonitor();
        mem->set_enabled(true);
        monitors_.push_back(std::move(mem));
    }
    if (options_.enable_hdd_monitor) {
        auto hdd = CreateHddMonitor();
        hdd->set_enabled(true);
        monitors_.push_back(std::move(hdd));
    }
    if (options_.enable_net_monitor) {
        auto net = CreateNetMonitor();
        net->set_enabled(true);
        monitors_.push_back(std::move(net));
    }
    if (options_.enable_ntp_monitor) {
        auto ntp = CreateNtpMonitor();
        ntp->set_enabled(true);
        monitors_.push_back(std::move(ntp));
    }
    if (options_.enable_process_monitor) {
        auto proc = CreateProcessMonitor();
        proc->set_enabled(true);
        monitors_.push_back(std::move(proc));
    }
    if (options_.enable_voltage_monitor) {
        auto volt = CreateVoltageMonitor();
        volt->set_enabled(true);
        monitors_.push_back(std::move(volt));
    }
}

void MonitorRegistry::StartGperfIfRequested() {
    if (options_.enable_cpu_profile && !options_.cpu_profile_filename.empty() &&
        GperfProfiler::IsCpuProfilingAvailable()) {
        gperf_profiler_.StartCpuProfile(options_.cpu_profile_filename);
        gperf_cpu_active_ = true;
    }
    if (options_.enable_heap_profile && !options_.heap_profile_filename.empty() &&
        GperfProfiler::IsHeapProfilingAvailable()) {
        gperf_profiler_.StartHeapProfile(options_.heap_profile_filename);
        gperf_heap_active_ = true;
    }
}

void MonitorRegistry::StopGperfIfActive() {
    if (gperf_cpu_active_) {
        gperf_profiler_.StopCpuProfile();
        gperf_cpu_active_ = false;
    }
    if (gperf_heap_active_) {
        gperf_profiler_.StopHeapProfile();
        gperf_heap_active_ = false;
    }
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
