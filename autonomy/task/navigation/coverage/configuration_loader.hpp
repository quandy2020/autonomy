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
 * @file configuration_loader.hpp
 * @brief Load the complete coverage runtime defaults from JSON.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_CONFIGURATION_LOADER_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_CONFIGURATION_LOADER_HPP_

#include "autonomy/task/navigation/coverage/configuration.hpp"
namespace autonomy::task::coverage {

/**
 * @brief Reads a runtime configuration from a JSON file.
 *
 * Unknown keys are ignored and missing keys keep the value from
 * @ref Configuration, so a partial file only overrides what it lists. Robot
 * limits are validated before the configuration is returned.
 *
 * @param[in] filename Path to the JSON configuration file.
 * @return Configuration with the file values applied.
 * @throws RequestException when the file cannot be read.
 * @throws RequestException when a robot limit fails validation.
 */
Configuration LoadConfiguration(const std::string& filename);

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_CONFIGURATION_LOADER_HPP_
