/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

/**
 * @file driver_params.hpp
 * @brief String-keyed driver parameters and typed parsers.
 *
 * YAML flattens vendor params into DriverParams. Drivers must parse into
 * member fields in their constructor (cold path); hot paths must not call
 * GetString / Parse* repeatedly.
 */

#ifndef AUTODRIVER_DRIVER_PARAMS_HPP_
#define AUTODRIVER_DRIVER_PARAMS_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

namespace autodriver {
namespace hardware {

/** @brief Flat key/value map from YAML sensor params (startup / Attach only). */
using DriverParams = std::unordered_map<std::string, std::string>;

/**
 * @brief Looks up a string parameter.
 * @param[in] params Driver parameter map.
 * @param[in] key Parameter name.
 * @param[in] default_value Value used when @p key is absent (default empty).
 * @return The stored string, or @p default_value.
 */
inline std::string GetString(const DriverParams& params, const std::string& key,
                             const std::string& default_value = {}) {
    const auto it = params.find(key);
    return it == params.end() ? default_value : it->second;
}

/**
 * @brief Reads an integer parameter, falling back to @p default_value.
 * @param[in] params Driver parameter map.
 * @param[in] key Parameter name.
 * @param[in] default_value Value used when missing or unparsable.
 * @return Parsed integer, or @p default_value.
 */
int ParseInt(const DriverParams& params, const std::string& key,
             int default_value);

/**
 * @brief Reads a CAN frame id parameter as uint32_t.
 * @param[in] params Driver parameter map.
 * @param[in] key Parameter name.
 * @param[in] default_value Value used when missing or unparsable.
 * @return Parsed CAN id, or @p default_value.
 */
std::uint32_t ParseCanId(const DriverParams& params, const std::string& key,
                         std::uint32_t default_value);

/**
 * @brief Reads a floating-point parameter via strtod.
 * @param[in] params Driver parameter map.
 * @param[in] key Parameter name.
 * @param[in] default_value Value used when missing or unparsable.
 * @return Parsed double, or @p default_value.
 */
double ParseDouble(const DriverParams& params, const std::string& key,
                   double default_value);

/**
 * @brief Reads a boolean parameter from common textual truth values.
 * @param[in] params Driver parameter map.
 * @param[in] key Parameter name.
 * @param[in] default_value Value used when missing or unrecognized.
 * @return Parsed boolean, or @p default_value.
 */
bool ParseBool(const DriverParams& params, const std::string& key,
               bool default_value);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_DRIVER_PARAMS_HPP_
