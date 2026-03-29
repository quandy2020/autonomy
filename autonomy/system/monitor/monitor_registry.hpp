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
#include <vector>

#include "autonomy/system/monitor/gperf_profiler.hpp"
#include "autonomy/system/monitor/monitor_base.hpp"
#include "autonomy/system/monitor/monitor_options.hpp"

namespace autonomy {
namespace system {
namespace monitor {

/**
 * 统一管理各监控器、Prometheus 暴露与 gperf 分析启停。
 * 支持参数选定：通过 MonitorOptions 选择启用的 monitor 与是否开启
 * Prometheus/gperf。
 */
class MonitorRegistry
{
public:
    explicit MonitorRegistry(MonitorOptions options);
    ~MonitorRegistry();

    MonitorRegistry(const MonitorRegistry&) = delete;
    MonitorRegistry& operator=(const MonitorRegistry&) = delete;

    /// 根据当前 options 创建并注册各 monitor，若 enable_prometheus 则启动
    /// Exposer
    void Start();

    /// 停止 Exposer、停止 gperf 分析
    void Stop();

    /// 对所有已启用的 monitor 执行一次采集（可由定时器周期性调用）
    void CollectAll();

    /// 手动添加外部 monitor（可选）
    void AddMonitor(std::unique_ptr<MonitorBase> monitor);

    const MonitorOptions& options() const {
        return options_;
    }
    MonitorOptions* mutable_options() {
        return &options_;
    }

private:
    void BuildMonitorsFromOptions();
    void StartGperfIfRequested();
    void StopGperfIfActive();

    MonitorOptions options_;
    GperfProfiler gperf_profiler_;
    std::vector<std::unique_ptr<MonitorBase>> monitors_;
    bool started_{false};
    bool gperf_cpu_active_{false};
    bool gperf_heap_active_{false};

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    struct PrometheusState;
    std::unique_ptr<PrometheusState> prometheus_;
#endif
};

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
