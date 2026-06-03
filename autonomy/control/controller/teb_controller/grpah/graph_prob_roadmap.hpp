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

#include <random>

#include "autonomy/common/macros.hpp"
#include "autonomy/control/controller/teb_controller/grpah/graph_search.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Probabilistic roadmap homotopy graph
 */
class ProbRoadmapGraph : public GraphSearchInterface
{
public:
    AUTONOMY_SHARED_PTR_DEFINITIONS(ProbRoadmapGraph);

    ProbRoadmapGraph(const TimedElasticBandConfig& cfg, HcController* hcp)
        : GraphSearchInterface(cfg, hcp) {}

    void CreateGraph(
        const Pose2D& start, const Pose2D& goal, double dist_to_obst,
        double obstacle_heading_threshold,
        const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
        bool free_goal_velocity = false);

private:
    std::mt19937 rnd_generator_;
};

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
