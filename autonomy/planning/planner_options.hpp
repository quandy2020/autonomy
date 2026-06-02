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

#pragma once

#include <string>

#include "autonomy/planning/proto/planning_options.pb.h"

namespace autonomy {
namespace common {
class LuaParameterDictionary;
}
namespace planning {

/**
 * @brief Load planning stack options from a Lua parameter dictionary.
 *
 * Parses config/planner/planner.lua (or equivalent) into proto::PlannerOptions,
 * including planner and smoother plugin lists, per-plugin tuning, global
 * costmap settings, and path post-processing flags.
 *
 * Typical usage:
 * @code
 *   auto resolver = std::make_unique<ConfigurationFileResolver>(config_dirs);
 *   auto dict = resolver->GetDictionary("planner.lua");
 *   auto options = autonomy::planning::LoadOptions(dict.get());
 * @endcode
 *
 * @param parameter_dictionary Root dictionary for the planner config file.
 * @return Populated PlannerOptions proto used by PlannerServer and
 * path smoother plugins (e.g. planning::utils::SimpleSmoother).
 */
proto::PlannerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

/**
 * Loads planner options from config/planner/planner.lua under
 * configuration_directory.
 */
proto::PlannerOptions CreateOptions(
    const std::string& configuration_directory);

}  // namespace planning
}  // namespace autonomy
