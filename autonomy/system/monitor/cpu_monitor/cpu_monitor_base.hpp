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

#include "autonomy/system/monitor/cpu_monitor/cpu_information.hpp"
#include "autonomy/system/monitor/cpu_monitor/cpu_usage_statistics.hpp"
#include "autonomy/system/monitor/monitor_base.hpp"

namespace autonomy {
namespace system {
namespace monitor {
namespace cpu_monitor {

/**
 * CPU 监控基类：从系统读取 CPU 信息与 tick，计算使用率，并可向 Prometheus 暴露指标。
 */
class CpuMonitorBase : public MonitorBase {
 public:
  std::string Name() const override { return "cpu"; }

  void Collect() override;

  void RegisterWithPrometheus(void* registry) override;

  const CpuInformation& info() const { return info_; }
  const CpuUsageStatistics& usage() const { return usage_; }

 protected:
  /// 子类可重写以提供平台特定信息
  virtual void FillCpuInformation(CpuInformation* info);
  /// 从系统读取当前 tick，返回各 cpu 行（cpu, cpu0, cpu1, ...）
  virtual bool ReadProcStat(std::vector<CpuTickCounts>* out);

  CpuInformation info_;
  CpuUsageStatistics usage_;
  std::vector<CpuTickCounts> prev_ticks_;
  bool first_collect_{true};

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
  struct PrometheusGauges;
  std::unique_ptr<PrometheusGauges> gauges_;
#endif
};

/// 创建当前平台适用的 CPU 监控器实例
std::unique_ptr<CpuMonitorBase> CreateCpuMonitor();

}  // namespace cpu_monitor
}  // namespace monitor
}  // namespace system
}  // namespace autonomy
