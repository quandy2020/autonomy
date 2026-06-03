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

#include <algorithm>

#include <boost/graph/adjacency_list.hpp>

#include "autonomy/control/controller/teb_controller/grpah/graph_search.hpp"
#include "autonomy/control/controller/teb_controller/hc_controller.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

void GraphSearchInterface::DepthFirst(
    HomotopyExplorationGraph& g,
    std::vector<HomotopyExplorationVertexType>& visited,
    const HomotopyExplorationVertexType& goal, double start_orientation,
    double goal_orientation,
    const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
    bool free_goal_velocity) {
    if ((int)hcp_->GetTrajectoryContainer().size() >=
        config_->homotopy.max_number_classes)
        return;

    HomotopyExplorationVertexType back = visited.back();

    HomotopyExplorationGraphAdjecencyIterator it, end;
    for (boost::tie(it, end) = boost::adjacent_vertices(back, g); it != end;
         ++it) {
        if (std::find(visited.begin(), visited.end(), *it) != visited.end())
            continue;

        if (*it == goal) {
            visited.push_back(*it);

            hcp_->AddAndInitNewTeb(
                visited.begin(), visited.end(),
                [&g = graph_](const auto& v) {
                    return GetPointFromHomotopyExplorationGraph(v, g);
                },
                start_orientation, goal_orientation, start_velocity);

            visited.pop_back();
            break;
        }
    }

    for (boost::tie(it, end) = boost::adjacent_vertices(back, g); it != end;
         ++it) {
        if (std::find(visited.begin(), visited.end(), *it) != visited.end() ||
            *it == goal)
            continue;

        visited.push_back(*it);

        DepthFirst(g, visited, goal, start_orientation, goal_orientation,
                   start_velocity, free_goal_velocity);

        visited.pop_back();
    }
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
