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

#include <boost/graph/adjacency_list.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/teb_controller/grpah/graph_prob_roadmap.hpp"
#include "autonomy/control/controller/teb_controller/hc_controller.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

void ProbRoadmapGraph::CreateGraph(
    const Pose2D& start, const Pose2D& goal, double dist_to_obst,
    double obstacle_heading_threshold,
    const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
    bool free_goal_velocity) {
    ClearExplorationGraph();
    if ((int)hcp_->GetTrajectoryContainer().size() >=
        config_->homotopy.max_number_classes)
        return;

    Point diff = Position(goal) - Position(start);
    double start_goal_dist = Norm(diff);

    if (start_goal_dist < config_->goal_tolerance.goal_position_tolerance) {
        ADEBUG << "ProbRoadmapGraph::CreateGraph(): xy-goal-tolerance already "
                  "reached.";
        if (hcp_->GetTrajectoryContainer().empty()) {
            AINFO << "ProbRoadmapGraph::CreateGraph(): Initializing a small "
                     "straight line to just correct orientation errors.";
            hcp_->AddAndInitNewTeb(start, goal, start_velocity, free_goal_velocity);
        }
        return;
    }
    Point normal = MakePoint(-diff.y, diff.x);
    normal = Normalized(normal);

    double area_width = config_->homotopy.roadmap_graph_area_width;

    std::uniform_real_distribution<double> distribution_x(
        0, start_goal_dist * config_->homotopy.roadmap_graph_area_length_scale);
    std::uniform_real_distribution<double> distribution_y(0, area_width);

    double phi = std::atan2(diff.y, diff.x);

    Point area_origin;
    if (config_->homotopy.roadmap_graph_area_length_scale != 1.0) {
        area_origin =
            Position(start) +
            (start_goal_dist *
             (0.5 * (1.0 - config_->homotopy.roadmap_graph_area_length_scale)) *
             Normalized(diff)) -
            (area_width * 0.5) * normal;
    } else {
        area_origin = Position(start) - (area_width * 0.5) * normal;
    }

    HomotopyExplorationVertexType start_vtx = boost::add_vertex(graph_);
    graph_[start_vtx] = Position(start);
    diff = Normalized(diff);

    for (int i = 0; i < config_->homotopy.roadmap_graph_sample_count; ++i) {
        Point sample =
            area_origin +
            Rotate2D(MakePoint(distribution_x(rnd_generator_),
                               distribution_y(rnd_generator_)),
                     phi);

        HomotopyExplorationVertexType v = boost::add_vertex(graph_);
        graph_[v] = sample;
    }

    HomotopyExplorationVertexType goal_vtx = boost::add_vertex(graph_);
    graph_[goal_vtx] = Position(goal);

    HomotopyExplorationVertexIterator it_i, end_i, it_j, end_j;
    for (boost::tie(it_i, end_i) = boost::vertices(graph_);
         it_i != boost::prior(end_i); ++it_i) {
        for (boost::tie(it_j, end_j) = boost::vertices(graph_); it_j != end_j; ++it_j) {
            if (it_i == it_j) {
                continue;
            }

            Point distij = Normalized(graph_[*it_j] - graph_[*it_i]);

            if (Dot(distij, diff) <= obstacle_heading_threshold) {
                continue;
            }

            bool collision = false;
            for (auto it_obst = hcp_->obstacles()->begin();
                 it_obst != hcp_->obstacles()->end(); ++it_obst) {
                if ((*it_obst)->CheckIntersection(graph_[*it_i], graph_[*it_j],
                                                 dist_to_obst)) {
                    collision = true;
                    break;
                }
            }
            if (collision) {
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
