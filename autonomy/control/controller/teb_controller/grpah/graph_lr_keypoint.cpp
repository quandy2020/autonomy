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

#include <cfloat>

#include <boost/graph/adjacency_list.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/teb_controller/grpah/graph_lr_keypoint.hpp"
#include "autonomy/control/controller/teb_controller/hc_controller.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

void LRKeyPointGraph::CreateGraph(
    const Pose2D& start, const Pose2D& goal, double dist_to_obst,
    double obstacle_heading_threshold,
    const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
    bool free_goal_velocity) {
    ClearExplorationGraph();
    if ((int)hcp_->GetTrajectoryContainer().size() >=
        config_->homotopy.max_number_classes)
        return;

    Point diff = Position(goal) - Position(start);

    if (Norm(diff) < config_->goal_tolerance.goal_position_tolerance) {
        ADEBUG << "LRKeyPointGraph::CreateGraph(): xy-goal-tolerance already "
                  "reached.";
        if (hcp_->GetTrajectoryContainer().empty()) {
            AINFO << "LRKeyPointGraph::CreateGraph(): Initializing a small "
                     "straight line to just correct orientation errors.";
            hcp_->AddAndInitNewTeb(start, goal, start_velocity, free_goal_velocity);
        }
        return;
    }

    Point normal = Normalized(MakePoint(-diff.y, diff.x)) * dist_to_obst;

    HomotopyExplorationVertexType start_vtx = boost::add_vertex(graph_);
    graph_[start_vtx] = Position(start);
    diff = Normalized(diff);

    std::pair<HomotopyExplorationVertexType, HomotopyExplorationVertexType>
        nearest_obstacle;
    double min_dist = DBL_MAX;

    if (hcp_->obstacles() != nullptr) {
        for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin();
             it_obst != hcp_->obstacles()->end(); ++it_obst) {
            Point start2obst = (*it_obst)->GetCentroid() - Position(start);
            double dist = Norm(start2obst);
            if (Dot(start2obst, diff) / dist < 0.1)
                continue;

            HomotopyExplorationVertexType u = boost::add_vertex(graph_);
            graph_[u] = (*it_obst)->GetCentroid() + normal;
            HomotopyExplorationVertexType v = boost::add_vertex(graph_);
            graph_[v] = (*it_obst)->GetCentroid() - normal;

            if (obstacle_heading_threshold && dist < min_dist) {
                min_dist = dist;
                nearest_obstacle.first = u;
                nearest_obstacle.second = v;
            }
        }
    }

    HomotopyExplorationVertexType goal_vtx = boost::add_vertex(graph_);
    graph_[goal_vtx] = Position(goal);

    HomotopyExplorationVertexIterator it_i, end_i, it_j, end_j;
    for (boost::tie(it_i, end_i) = boost::vertices(graph_); it_i != end_i - 1;
         ++it_i) {
        for (boost::tie(it_j, end_j) = boost::vertices(graph_); it_j != end_j;
             ++it_j) {
            if (it_i == it_j)
                continue;

            Point distij = Normalized(graph_[*it_j] - graph_[*it_i]);
            if (Dot(distij, diff) <= obstacle_heading_threshold)
                continue;

            if (obstacle_heading_threshold && *it_i == start_vtx &&
                min_dist != DBL_MAX) {
                if (*it_j == nearest_obstacle.first ||
                    *it_j == nearest_obstacle.second) {
                    Point keypoint_dist = Normalized(graph_[*it_j] - Position(start));
                    Point start_orient_vec =
                        MakePoint(std::cos(start.theta), std::sin(start.theta));
                    if (Dot(start_orient_vec, keypoint_dist) <=
                        obstacle_heading_threshold) {
                        ADEBUG << "LRKeyPointGraph::CreateGraph() - deleted "
                                  "edge: limit_obstacle_heading";
                        continue;
                    }
                }
            }

            if (hcp_->obstacles() != nullptr) {
                bool collision = false;
                for (ObstContainer::const_iterator it_obst =
                         hcp_->obstacles()->begin();
                     it_obst != hcp_->obstacles()->end(); ++it_obst) {
                    if ((*it_obst)->CheckIntersection(graph_[*it_i],
                                                      graph_[*it_j],
                                                      0.5 * dist_to_obst)) {
                        collision = true;
                        break;
                    }
                }
                if (collision)
                    continue;
            }

            boost::add_edge(*it_i, *it_j, graph_);
        }
    }

    std::vector<HomotopyExplorationVertexType> visited;
    visited.push_back(start_vtx);
    DepthFirst(graph_, visited, goal_vtx, start.theta, goal.theta,
               start_velocity, free_goal_velocity);
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
