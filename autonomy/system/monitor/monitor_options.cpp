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

namespace autonomy {
namespace system {
namespace monitor {

MonitorOptions MonitorOptions::Default() {
    MonitorOptions opts;
    opts.enable_cpu_monitor = true;
    opts.enable_mem_monitor = true;
    opts.enable_prometheus = true;
    opts.prometheus_bind_address = "0.0.0.0:9090";
    opts.prometheus_metrics_prefix = "autonomy_system";
    opts.collect_interval_sec = 1.0;
    return opts;
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
