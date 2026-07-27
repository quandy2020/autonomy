/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

std::unordered_map<std::string, LngLat> BuildNodeIndex(const std::vector<GraphNode>& nodes);
std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>
BuildAdjacency(const std::vector<GraphNode>& nodes, const std::vector<GraphEdge>& edges,
               const std::unordered_map<std::string, LngLat>& nodeIndex);

class GraphPathfinder
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(GraphPathfinder)

    explicit GraphPathfinder(const RoadGraph& graph);

    std::optional<std::vector<std::string>> FindPath(
        const std::string& startNodeId, const std::string& endNodeId,
        const std::unordered_set<std::string>& excludeNodeIds = {});

    std::optional<std::string> FindNearestNode(const LngLat& position);

    std::optional<std::vector<std::string>> PlanCircuit(
        const std::vector<std::string>& nodeIds);

    std::optional<std::vector<std::string>> PlanChinesePostmanCircuit();
    std::optional<std::vector<std::string>> PlanGreedyCircuit(
        const std::vector<std::string>& nodeIds);

private:
    double Heuristic(const std::string& nodeId, const LngLat& endCoords) const;

    RoadGraph graph_;
    std::unordered_map<std::string, LngLat> nodeIndex_;
    std::unordered_map<std::string, std::vector<std::pair<std::string, double>>> adjacency_;
};

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy
