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

#ifndef AUTODRIVER_CONFIG_LOADER_HPP_
#define AUTODRIVER_CONFIG_LOADER_HPP_

#include <string>

#include "autodriver/config.hpp"

namespace autodriver {

inline constexpr const char* kDefaultConfigBasename = "autodriver_hardware.yaml";

// Loads config/autodriver_hardware.yaml from WorkRoot() (see AUTODRIVER_PATH).
Config LoadConfig();

// Loads config/<basename> under WorkRoot(), or an absolute path.
// Empty basename defaults to autodriver_hardware.yaml.
Config LoadConfig(const std::string& config_basename);

// Optional root override (AUTODRIVER_PATH) plus basename under config/.
Config LoadConfig(const std::string& configuration_directory,
                  const std::string& config_basename);

}  // namespace autodriver

#endif  // AUTODRIVER_CONFIG_LOADER_HPP_
