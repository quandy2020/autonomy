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
#include <string>
#include <vector>

namespace autonomy {
namespace system {
namespace monitor {
namespace cpu_monitor {

struct CpuTemperatureReading {
    std::string label;
    double celsius{0.0};
};

struct LoadAverage {
    double load_1{0.0};
    double load_5{0.0};
    double load_15{0.0};
};

struct CpuInformation {
    std::string model_name;
    uint32_t num_cores{0};
    uint64_t freq_hz{0};
    std::string vendor;  // "intel", "arm", "unknown" 等，来自 /proc/cpuinfo
};

}  // namespace cpu_monitor
}  // namespace monitor
}  // namespace system
}  // namespace autonomy
