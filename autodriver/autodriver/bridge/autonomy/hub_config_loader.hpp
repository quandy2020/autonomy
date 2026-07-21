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
 * @brief Loads HubConfig from autonomy Lua configuration files.
 */

#ifndef AUTODRIVER_BRIDGE_AUTONOMY_HUB_CONFIG_LOADER_HPP_
#define AUTODRIVER_BRIDGE_AUTONOMY_HUB_CONFIG_LOADER_HPP_

#include <string>

#include "autodriver/config/hub_config.hpp"

namespace autodriver {
namespace bridge {

/**
 * @brief Loads AUTODRIVER table from a Lua file (+ common.lua).
 * @param configuration_directory Override config root; empty uses install share.
 * @param config_basename Relative path, e.g. driver/autodriver_hardware.lua.
 */
HubConfig LoadHubConfigFromDirectory(
  const std::string & configuration_directory = {},
  const std::string & config_basename = "driver/autodriver.lua");

}  // namespace bridge
}  // namespace autodriver

#endif  // AUTODRIVER_BRIDGE_AUTONOMY_HUB_CONFIG_LOADER_HPP_
