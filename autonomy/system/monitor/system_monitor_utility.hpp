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

namespace autonomy {
namespace system {
namespace monitor {

struct ThermalZone {
    std::string type;
    std::string label;
    std::string temperature_path;
};

/// Linux sysfs 热区发现（x86 / ARM 通用，按 type 前缀匹配）
class SystemMonitorUtility
{
public:
    static void ToLowercase(std::string* s);

    /// @param type_prefix 如 "cpu-therm"，与 thermal zone type 做小写前缀比较
    static void GetThermalZonesByTypePrefix(
        const std::string& type_prefix, std::vector<ThermalZone>* out);

    /// Intel coretemp hwmon temp*_input（x86 常见）
    static void GetIntelCoretempInputs(std::vector<ThermalZone>* out);
};

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
