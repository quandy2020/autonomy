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

#include "autonomy/system/monitor/monitor_options.hpp"

#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/system/monitor/ops_types.hpp"
#include "autonomy/system/proto/monitor_options.pb.h"
#include "autonomy/task/common/names.hpp"
#include "autonomy/task/teleop/constants.hpp"

namespace autonomy {
namespace system {
namespace monitor {
namespace {

void ApplyDefaultWatches(MonitorOptions* opts) {
    if (opts == nullptr) {
        return;
    }
    if (opts->channel_watches.empty()) {
        opts->channel_watches.push_back(
            {task::teleop::kCommandVelocityTopic, 1.0, 0.0});
        opts->channel_watches.push_back({task::kTeleopGoal, 5.0, 0.0});
        opts->channel_watches.push_back({task::kTeleopFeedback, 5.0, 0.0});
    }
    if (opts->latency_watches.empty()) {
        opts->latency_watches.push_back(
            {task::teleop::kCommandVelocityTopic, 0.5});
    }
}

void ApplyDefaultCriticalProcesses(MonitorOptions* opts) {
    if (opts == nullptr || !opts->critical_processes.empty()) {
        return;
    }
    opts->critical_processes.push_back(
        {"monitor", "autonomy.monitor", "respawn"});
    opts->critical_processes.push_back(
        {"planning", "autonomy.planning", "respawn"});
    opts->critical_processes.push_back(
        {"control", "autonomy.control", "respawn"});
    opts->critical_processes.push_back({"task", "autonomy.task", "respawn"});
    opts->critical_processes.push_back(
        {"bridge", "autonomy.bridge", "respawn"});
}

MonitorOptions FromProto(const ::autonomy::system::proto::MonitorOptions& p) {
    MonitorOptions opts = MonitorOptions::Default();
    opts.enable_cpu_monitor = p.enable_cpu_monitor();
    opts.enable_gpu_monitor = p.enable_gpu_monitor();
    opts.enable_mem_monitor = p.enable_mem_monitor();
    opts.enable_hdd_monitor = p.enable_hdd_monitor();
    opts.enable_net_monitor = p.enable_net_monitor();
    opts.enable_ntp_monitor = p.enable_ntp_monitor();
    opts.enable_process_monitor = p.enable_process_monitor();
    opts.enable_voltage_monitor = p.enable_voltage_monitor();
    opts.enable_channel_monitor = p.enable_channel_monitor();
    opts.enable_latency_monitor = p.enable_latency_monitor();
    opts.enable_hazard_monitor = p.enable_hazard_monitor();
    opts.enable_mrm_handler = p.enable_mrm_handler();
    opts.enable_prometheus = p.enable_prometheus();
    if (!p.prometheus_bind_address().empty()) {
        opts.prometheus_bind_address = p.prometheus_bind_address();
    }
    if (!p.prometheus_metrics_prefix().empty()) {
        opts.prometheus_metrics_prefix = p.prometheus_metrics_prefix();
    }
    if (p.collect_interval_sec() > 0.0) {
        opts.collect_interval_sec = p.collect_interval_sec();
    }
    opts.enable_cpu_profile = p.enable_cpu_profile();
    opts.cpu_profile_filename = p.cpu_profile_filename();
    opts.enable_heap_profile = p.enable_heap_profile();
    opts.heap_profile_filename = p.heap_profile_filename();

    opts.channel_watches.clear();
    for (const auto& w : p.channel_watches()) {
        if (w.channel().empty()) {
            continue;
        }
        opts.channel_watches.push_back(
            {w.channel(), w.timeout_sec(), w.min_rate_hz()});
    }
    opts.latency_watches.clear();
    for (const auto& w : p.latency_watches()) {
        if (w.channel().empty()) {
            continue;
        }
        opts.latency_watches.push_back({w.channel(), w.max_age_sec()});
    }
    if (p.has_mrm()) {
        if (!p.mrm().cmd_vel_channel().empty()) {
            opts.mrm.cmd_vel_channel = p.mrm().cmd_vel_channel();
        }
        opts.mrm.emergency_stop_on_error = p.mrm().emergency_stop_on_error();
    }
    opts.critical_processes.clear();
    for (const auto& cp : p.critical_processes()) {
        if (cp.name().empty()) {
            continue;
        }
        CriticalProcessOptions o;
        o.name = cp.name();
        o.match = cp.match().empty() ? cp.name() : cp.match();
        o.restart_hint =
            cp.restart_hint().empty() ? "respawn" : cp.restart_hint();
        opts.critical_processes.push_back(std::move(o));
    }
    // health_snapshot_path "-" disables publish; otherwise publish (default on).
    if (p.health_snapshot_path() == "-") {
        opts.publish_health_snapshot = false;
        opts.health_snapshot_path.clear();
    } else {
        opts.publish_health_snapshot = true;
        opts.health_snapshot_path = p.health_snapshot_path();
    }
    ApplyDefaultWatches(&opts);
    ApplyDefaultCriticalProcesses(&opts);
    return opts;
}

}  // namespace

MonitorOptions MonitorOptions::Default() {
    MonitorOptions opts;
    opts.enable_cpu_monitor = true;
    opts.enable_mem_monitor = true;
    opts.enable_process_monitor = true;
    opts.enable_prometheus = true;
    opts.prometheus_bind_address = "0.0.0.0:9090";
    opts.prometheus_metrics_prefix = "autonomy_system";
    opts.collect_interval_sec = 1.0;
    opts.publish_health_snapshot = true;
    ApplyDefaultWatches(&opts);
    ApplyDefaultCriticalProcesses(&opts);
    return opts;
}

MonitorOptions LoadMonitorOptions(const std::string& conf_file) {
    ::autonomy::system::proto::MonitorOptions pb;
    const std::string file =
        conf_file.empty() ? std::string("monitor.pb.txt") : conf_file;
    if (!common::LoadModuleConf("system", file, &pb)) {
        AWARN << "Monitor config not loaded (" << file
              << ") — using defaults";
        return MonitorOptions::Default();
    }
    return FromProto(pb);
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
