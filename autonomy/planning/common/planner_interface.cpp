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

#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/plugins/dijkstra/dijkstra_planner.hpp"
#include "autonomy/planning/plugins/navfn/navfn_planner.hpp"

namespace autonomy {
namespace planning {
namespace common {

proto::PlannerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::PlannerOptions options;
    *options.mutable_dijkstra() = plugins::dijkstra::CreateDijkstraPlannerOptions(parameter_dictionary->GetDictionary("dijkstra_planner").get());
    *options.mutable_navfn() = plugins::navfn::CreateNavFnPlannerOptions(parameter_dictionary->GetDictionary("navfn_planner").get());
    return options;
}

}  // namespace common
}  // namespace planning
}  // namespace autonomy


