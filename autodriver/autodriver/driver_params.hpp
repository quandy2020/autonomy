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

/**
 * @file
 * @brief String-keyed driver parameters and typed parsers.
 */

#ifndef AUTODRIVER_DRIVER_PARAMS_HPP_
#define AUTODRIVER_DRIVER_PARAMS_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

namespace autodriver {
namespace hardware {

/** @brief Flat key/value map from YAML sensor params. */
using DriverParams = std::unordered_map<std::string, std::string>;

/**
 * @brief Read a string parameter with a default fallback.
 * @param params Driver parameter map from configuration.
 * @param key Parameter name to look up.
 * @param default_value Value returned when the key is absent.
 * @return Stored string value or default_value.
 */
inline std::string GetString(const DriverParams& params, const std::string& key,
                             const std::string& default_value = {}) {
    const auto it = params.find(key);
    return it == params.end() ? default_value : it->second;
}

/**
 * @brief Parse a decimal or 0x-prefixed hex integer parameter.
 * @param params Driver parameter map from configuration.
 * @param key Parameter name to look up.
 * @param default_value Value returned when the key is absent or invalid.
 * @return Parsed integer value.
 */
int ParseInt(const DriverParams& params, const std::string& key,
             int default_value);

/**
 * @brief Parse a CAN identifier (decimal or hex).
 * @param params Driver parameter map from configuration.
 * @param key Parameter name to look up.
 * @param default_value Value returned when the key is absent or invalid.
 * @return Parsed CAN id.
 */
std::uint32_t ParseCanId(const DriverParams& params, const std::string& key,
                         std::uint32_t default_value);

/**
 * @brief Parse a floating-point parameter.
 * @param params Driver parameter map from configuration.
 * @param key Parameter name to look up.
 * @param default_value Value returned when the key is absent or invalid.
 * @return Parsed floating-point value.
 */
double ParseDouble(const DriverParams& params, const std::string& key,
                   double default_value);

/**
 * @brief Parse a boolean parameter (true/false, 1/0, yes/no, on/off).
 * @param params Driver parameter map from configuration.
 * @param key Parameter name to look up.
 * @param default_value Value returned when the key is absent or unrecognized.
 * @return Parsed boolean value.
 */
bool ParseBool(const DriverParams& params, const std::string& key,
               bool default_value);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_DRIVER_PARAMS_HPP_
