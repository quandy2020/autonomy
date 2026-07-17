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

#include <string>
#include <vector>

#include "autonomy/system/monitor/ops_types.hpp"

namespace autonomy {
namespace system {
namespace monitor {

struct MrmHandlerOptions {
    std::string cmd_vel_channel{"/cmd_vel"};
    bool emergency_stop_on_error{true};
};

/**
 * 可选的监控项与 Prometheus/gperf 参数选定。
 * 用于控制启用哪些 monitor 以及可视化/分析相关配置。
 */
struct MonitorOptions {
    // ---------- 主机监控 ----------
    bool enable_cpu_monitor{true};
    bool enable_gpu_monitor{false};
    bool enable_mem_monitor{true};
    bool enable_hdd_monitor{false};
    bool enable_net_monitor{false};
    bool enable_ntp_monitor{false};
    bool enable_process_monitor{false};
    bool enable_voltage_monitor{false};

    // ---------- Autolink 运维（参考 Autoware topic/pipeline/MRM） ----------
    bool enable_channel_monitor{true};
    bool enable_latency_monitor{true};
    bool enable_hazard_monitor{true};
    bool enable_mrm_handler{false};
    std::vector<ChannelWatchOptions> channel_watches;
    std::vector<LatencyWatchOptions> latency_watches;
    MrmHandlerOptions mrm;

    // ---------- Prometheus 可视化 ----------
    bool enable_prometheus{true};
    std::string prometheus_bind_address{"0.0.0.0:9090"};
    std::string prometheus_metrics_prefix{"autonomy_system"};

    // ---------- gperftools 分析（参数选定） ----------
    bool enable_cpu_profile{false};
    std::string cpu_profile_filename{};
    bool enable_heap_profile{false};
    std::string heap_profile_filename{};

    // ---------- 采集间隔（秒） ----------
    double collect_interval_sec{1.0};

    static MonitorOptions Default();
};

/// 从 Lua 配置加载；文件缺失或解析失败时返回 Default() 并写日志。
MonitorOptions LoadMonitorOptions(const std::string& configuration_directory,
                                  const std::string& configuration_basename);

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
