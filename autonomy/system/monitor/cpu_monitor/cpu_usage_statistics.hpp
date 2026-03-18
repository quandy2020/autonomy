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

#include <cstdint>
#include <vector>

namespace autonomy {
namespace system {
namespace monitor {
namespace cpu_monitor {

/**
 * 单核或整体的 CPU 时间计数（与 /proc/stat 对应）。
 */
struct CpuTickCounts {
  uint64_t user{0};
  uint64_t nice{0};
  uint64_t system{0};
  uint64_t idle{0};
  uint64_t iowait{0};
  uint64_t irq{0};
  uint64_t softirq{0};
  uint64_t steal{0};
  uint64_t guest{0};
  uint64_t guest_nice{0};

  uint64_t Total() const { return user + nice + system + idle + iowait + irq + softirq + steal + guest + guest_nice; }
  uint64_t Idle() const { return idle + iowait; }
  uint64_t NonIdle() const { return Total() - Idle(); }
};

/**
 * 各核及整体的 CPU 使用率 [0, 100]。
 */
struct CpuUsageStatistics {
  /// 整体使用率（所有核平均）
  double total_usage_percent{0.0};
  /// 每核使用率，下标 0 为 cpu0，与 /proc/stat 一致
  std::vector<double> per_core_usage_percent;
};

/// 根据前后两次 tick 计数计算使用率
void ComputeUsage(const std::vector<CpuTickCounts>& prev, const std::vector<CpuTickCounts>& curr,
                  CpuUsageStatistics* out);

}  // namespace cpu_monitor
}  // namespace monitor
}  // namespace system
}  // namespace autonomy
