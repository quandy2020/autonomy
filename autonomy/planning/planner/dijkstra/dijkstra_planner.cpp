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

#include "autonomy/planning/planner/dijkstra/dijkstra_planner.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace dijkstra {

DijkstraPlanner::DijkstraPlanner(
    const proto::PlannerOptions& options, const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap)
    : navfn::NavfnPlanner(options, name, std::move(costmap)) {
    const auto& dijkstra_opts = options_.dijkstra();
    if (dijkstra_opts.tolerance() > 0.0) {
        SetTolerance(dijkstra_opts.tolerance());
    }
    SetAllowUnknown(dijkstra_opts.allow_unknown());
    SetUseFinalApproachOrientation(
        dijkstra_opts.use_final_approach_orientation());
    SetUseAstar(false);
}

}  // namespace dijkstra
}  // namespace planner
}  // namespace planning
}  // namespace autonomy

using autonomy::planning::common::GlobalPlanner;
using autonomy::planning::planner::dijkstra::DijkstraPlanner;

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(DijkstraPlanner, GlobalPlanner);
