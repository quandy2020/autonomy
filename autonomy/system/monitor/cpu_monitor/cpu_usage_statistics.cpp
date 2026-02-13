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

#include "autonomy/system/monitor/cpu_monitor/cpu_usage_statistics.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace system {
namespace monitor {
namespace cpu_monitor {

namespace {

double UsagePercentFromTicks(const CpuTickCounts& prev, const CpuTickCounts& curr) {
    const uint64_t total_delta = curr.Total() - prev.Total();
    if (total_delta == 0)
        return 0.0;
    const uint64_t idle_delta = curr.Idle() - prev.Idle();
    const uint64_t used = total_delta - idle_delta;
    return 100.0 * static_cast<double>(used) / static_cast<double>(total_delta);
}

}  // namespace

void ComputeUsage(const std::vector<CpuTickCounts>& prev, const std::vector<CpuTickCounts>& curr,
                  CpuUsageStatistics* out) {
    if (out == nullptr || prev.size() != curr.size() || prev.empty())
        return;
    out->per_core_usage_percent.resize(curr.size());
    double sum = 0.0;
    for (size_t i = 0; i < curr.size(); ++i) {
        double pct = UsagePercentFromTicks(prev[i], curr[i]);
        out->per_core_usage_percent[i] = std::min(100.0, std::max(0.0, pct));
        sum += out->per_core_usage_percent[i];
    }
    out->total_usage_percent = curr.size() ? (sum / static_cast<double>(curr.size())) : 0.0;
}

}  // namespace cpu_monitor
}  // namespace monitor
}  // namespace system
}  // namespace autonomy
