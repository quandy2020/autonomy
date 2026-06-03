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

#include <complex>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/utility.hpp>

#include "autonomy/control/controller/teb_controller/config.hpp"
#include "autonomy/control/controller/teb_controller/pose2d_utils.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

class HcController;

using HomotopyExplorationGraph =
    boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, Point,
                          boost::no_property>;
using HomotopyExplorationVertexType =
    boost::graph_traits<HomotopyExplorationGraph>::vertex_descriptor;
using HomotopyExplorationGraphEdgeType =
    boost::graph_traits<HomotopyExplorationGraph>::edge_descriptor;
using HomotopyExplorationVertexIterator =
    boost::graph_traits<HomotopyExplorationGraph>::vertex_iterator;
using HomotopyExplorationGraphEdgeIterator =
    boost::graph_traits<HomotopyExplorationGraph>::edge_iterator;
using HomotopyExplorationGraphAdjecencyIterator =
    boost::graph_traits<HomotopyExplorationGraph>::adjacency_iterator;

inline std::complex<long double> GetCplxFromHomotopyExplorationGraph(
    HomotopyExplorationVertexType vert_descriptor,
    const HomotopyExplorationGraph& graph) {
    const Point& pos = graph[vert_descriptor];
    return std::complex<long double>(pos.x, pos.y);
}

inline const Point& GetPointFromHomotopyExplorationGraph(
    HomotopyExplorationVertexType vert_descriptor,
    const HomotopyExplorationGraph& graph) {
    return graph[vert_descriptor];
}

/**
 * @brief Homotopy graph search interface
 */
class GraphSearchInterface
{
public:
    virtual void CreateGraph(
        const Pose2D& start, const Pose2D& goal, double dist_to_obst,
        double obstacle_heading_threshold,
        const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
        bool free_goal_velocity = false) = 0;

    void ClearExplorationGraph() {
        graph_.clear();
    }

    HomotopyExplorationGraph graph_;

protected:
    HcController* hcp_;
    const TimedElasticBandConfig* config_;
};

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
