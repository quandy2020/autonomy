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

#include "autonomy/system/monitor/system_monitor_utility.hpp"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <regex>
#include <string>

namespace fs = std::filesystem;

namespace autonomy {
namespace system {
namespace monitor {

void SystemMonitorUtility::ToLowercase(std::string* s) {
    if (s == nullptr)
        return;
    for (char& c : *s)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
}

void SystemMonitorUtility::GetThermalZonesByTypePrefix(
    const std::string& type_prefix, std::vector<ThermalZone>* out) {
    if (out == nullptr)
        return;

    std::string want = type_prefix;
    ToLowercase(&want);
    if (want.empty())
        return;

    const fs::path root("/sys/class/thermal");
    if (!fs::exists(root) || !fs::is_directory(root))
        return;

    for (const auto& entry : fs::directory_iterator(root)) {
        if (!entry.is_directory())
            continue;
        const std::string dir = entry.path().generic_string();
        if (!std::regex_match(dir, std::regex(".*/thermal_zone(\\d+)")))
            continue;

        std::string type;
        {
            std::ifstream ifs(entry.path() / "type");
            if (ifs)
                std::getline(ifs, type);
        }
        std::string type_lower = type;
        ToLowercase(&type_lower);
        if (type_lower.compare(0, want.size(), want) != 0)
            continue;

        ThermalZone zone;
        zone.type = type;
        zone.label = entry.path().filename().string();
        zone.temperature_path = (entry.path() / "temp").generic_string();
        out->push_back(std::move(zone));
    }
}

void SystemMonitorUtility::GetIntelCoretempInputs(std::vector<ThermalZone>* out) {
    if (out == nullptr)
        return;

    const fs::path root("/sys/devices/platform/coretemp.0");
    if (!fs::exists(root))
        return;

    std::error_code ec;
    for (const auto& entry :
         fs::recursive_directory_iterator(root, fs::directory_options::skip_permission_denied,
                                          ec)) {
        if (ec)
            break;
        if (!entry.is_regular_file())
            continue;
        const std::string path = entry.path().generic_string();
        std::cmatch match;
        if (!std::regex_match(path.c_str(), match, std::regex(".*temp(\\d+)_input")))
            continue;
        ThermalZone zone;
        zone.type = "coretemp";
        zone.label = "core" + std::string(match[1].first, match[1].second);
        zone.temperature_path = path;
        out->push_back(std::move(zone));
    }
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
