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

#include "autonomy/system/proto/autonomy_options.pb.h"
#include "autonomy/common/lua_parameter_dictionary.hpp"

#include "autonomy/bridge/common/bridge_interface.hpp"
#include "autonomy/localization/common/localization_interface.hpp"
#include "autonomy/map/common/map_interface.hpp"
#include "autonomy/prediction/common/prediction_interface.hpp"
#include "autonomy/perception/common/perception_interface.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/tasks/common/task_interface.hpp"
#include "autonomy/transform/common/transform_interface.hpp"
#include "autonomy/visualization/common/visualization_interface.hpp"

namespace autonomy {
namespace system { 
namespace common {

proto::AutonomyOptions CreateOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);

proto::AutonomyOptions LoadOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary);
    
}   // namespace common
}   // namespace system
}   // namespace autonomy