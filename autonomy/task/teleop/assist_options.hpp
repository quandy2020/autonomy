/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include "autonomy/task/teleop/mppi_assist.hpp"

namespace autonomy::task::teleop {

// Default relative path under the task config directory.
constexpr char kDefaultTeleopAssistConfigRelPath[] = "task/teleop_assist.lua";

/**
 * @brief Load teleop assist options from Lua configuration
 * @param config_directory Base config search path
 * @param relative_path Config file relative to config_directory
 * @return Options with enabled=false if the file is missing or invalid
 */
TeleopMppiAssist::Options LoadTeleopAssistOptions(
    const std::string& config_directory,
    const std::string& relative_path = kDefaultTeleopAssistConfigRelPath);

}  // namespace autonomy::task::teleop
