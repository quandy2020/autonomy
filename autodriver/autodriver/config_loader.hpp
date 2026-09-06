/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Configuration loader (YAML only).
 */

#ifndef AUTODRIVER_CONFIG_LOADER_HPP_
#define AUTODRIVER_CONFIG_LOADER_HPP_

#include <string>

#include "autodriver/config.hpp"

namespace autodriver {

// Default configuration file name under config/.
inline constexpr const char* kDefaultConfigBasename = "autodriver_hardware.yaml";

/**
 * @brief Loads the default autodriver_hardware.yaml config.
 */
Config LoadConfig();

/**
 * @brief Loads a YAML config by basename from the default configuration directory.
 */
Config LoadConfig(const std::string& config_basename);

/**
 * @brief Loads YAML from `{configuration_directory}/config/{basename}`
 *        (with install-tree fallback). Basename should be `.yaml` / `.yml`.
 */
Config LoadConfig(const std::string& configuration_directory,
                  const std::string& config_basename);

}  // namespace autodriver

#endif  // AUTODRIVER_CONFIG_LOADER_HPP_
