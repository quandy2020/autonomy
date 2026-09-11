/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file config_loader.hpp
 * @brief Configuration loader (YAML only).
 */

#ifndef AUTODRIVER_CONFIG_LOADER_HPP_
#define AUTODRIVER_CONFIG_LOADER_HPP_

#include <string>

#include "autodriver/config.hpp"

namespace autodriver {

/** @brief Default configuration file name under config/. */
inline constexpr const char* kDefaultConfigBasename = "autodriver_hardware.yaml";

/**
 * @brief Loads the default autodriver_hardware.yaml config.
 * @return Parsed Config (empty/default fields when the file is missing).
 */
Config LoadConfig();

/**
 * @brief Loads a YAML config by basename from the default configuration directory.
 *
 * Basename is typically `autodriver_hardware.yaml`. Camera vendor device
 * params are merged via each sensor's `params_file` (under config/camera/).
 *
 * @param[in] config_basename File name under `config/` (not a full path).
 * @return Parsed Config.
 */
Config LoadConfig(const std::string& config_basename);

/**
 * @brief Loads YAML from `{configuration_directory}/config/{basename}`
 *        (with install-tree fallback).
 * @param[in] configuration_directory Package root that contains `config/`.
 * @param[in] config_basename File name under `config/`.
 * @return Parsed Config.
 */
Config LoadConfig(const std::string& configuration_directory,
                  const std::string& config_basename);

}  // namespace autodriver

#endif  // AUTODRIVER_CONFIG_LOADER_HPP_
