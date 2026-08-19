/*
 * Copyright 2026 Autodriver contributors
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

#ifndef AUTODRIVER_DRIVER_PARAMS_HPP_
#define AUTODRIVER_DRIVER_PARAMS_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

namespace autodriver {
namespace hardware {

using DriverParams = std::unordered_map<std::string, std::string>;

inline std::string GetString(const DriverParams& params, const std::string& key,
                             const std::string& default_value = {}) {
    const auto it = params.find(key);
    return it == params.end() ? default_value : it->second;
}

int ParseInt(const DriverParams& params, const std::string& key,
             int default_value);
std::uint32_t ParseCanId(const DriverParams& params, const std::string& key,
                         std::uint32_t default_value);
double ParseDouble(const DriverParams& params, const std::string& key,
                   double default_value);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_DRIVER_PARAMS_HPP_
