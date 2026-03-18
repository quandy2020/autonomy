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

#include "autonomy/system/monitor/monitor_base.hpp"

namespace autonomy {
namespace system {
namespace monitor {

class ProcessMonitor : public MonitorBase {
 public:
  std::string Name() const override { return "process"; }
  void Collect() override;
  void RegisterWithPrometheus(void* registry) override;
  static std::unique_ptr<ProcessMonitor> Create() { return std::make_unique<ProcessMonitor>(); }
};

inline std::unique_ptr<ProcessMonitor> CreateProcessMonitor() { return ProcessMonitor::Create(); }

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
