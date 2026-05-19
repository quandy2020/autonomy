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

#include "autonomy/planning/planner_options.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/common/config.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/map/map_options.hpp"

namespace autonomy {
namespace planning {

namespace {

proto::NavFnPlanner LoadNavFnPlannerOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::NavFnPlanner options;
    options.set_tolerance(parameter_dictionary->GetDouble("tolerance"));
    options.set_use_astar(parameter_dictionary->GetBool("use_astar"));
    options.set_allow_unknown(parameter_dictionary->GetBool("allow_unknown"));
    options.set_use_final_approach_orientation(
        parameter_dictionary->GetBool("use_final_approach_orientation"));
    return options;
}

}  // namespace

proto::PlannerOptions LoadOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::PlannerOptions options;
    options.set_expected_planner_frequency(
        parameter_dictionary->GetDouble("expected_planner_frequency"));
    *options.mutable_navfn() = LoadNavFnPlannerOptions(
        parameter_dictionary->GetDictionary("navfn_planner").get());
    *options.mutable_costmap() = map::CreateCostmap2DOptions(
        parameter_dictionary->GetDictionary("costmap").get());
    return options;
}

}  // namespace planning
}  // namespace autonomy
