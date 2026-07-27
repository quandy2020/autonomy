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

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "autonomy/map/strata/navigation/graph_pathfinder.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

std::unordered_map<std::string, LngLat> BuildNodeIndex(
    const std::vector<GraphNode>& nodes) {
    std::unordered_map<std::string, LngLat> index;
    for (const auto& node : nodes) {
        index[node.id] = node.coordinates;
    }
    return index;
}

std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>
BuildAdjacency(const std::vector<GraphNode>& nodes, const std::vector<GraphEdge>& edges,
               const std::unordered_map<std::string, LngLat>& nodeIndex) {
    std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>> adj;
    for (const auto& node : nodes) {
        adj[node.id] = {};
    }
    for (const auto& edge : edges) {
        if (!nodeIndex.count(edge.from) || !nodeIndex.count(edge.to)) {
            continue;
        }
        adj[edge.from].push_back({edge.to, edge.id});
        adj[edge.to].push_back({edge.from, edge.id});
    }
    return adj;
}

GraphPathfinder::GraphPathfinder(const RoadGraph& graph) : graph_(graph) {
    nodeIndex_ = BuildNodeIndex(graph.nodes);
    adjacency_.clear();
    for (const auto& node : graph.nodes) {
        adjacency_[node.id] = {};
    }
    for (const auto& edge : graph.edges) {
        if (!nodeIndex_.count(edge.from) || !nodeIndex_.count(edge.to)) {
            continue;
        }
        adjacency_[edge.from].push_back({edge.to, edge.weight});
        adjacency_[edge.to].push_back({edge.from, edge.weight});
    }
}

double GraphPathfinder::Heuristic(const std::string& nodeId,
                                  const LngLat& endCoords) const {
    const auto it = nodeIndex_.find(nodeId);
    if (it == nodeIndex_.end()) {
        return 0.;
    }
    const double deltaLng = it->second.x - endCoords.x;
    const double deltaLat = it->second.y - endCoords.y;
    return std::sqrt(deltaLng * deltaLng + deltaLat * deltaLat);
}

std::optional<std::vector<std::string>> GraphPathfinder::FindPath(
    const std::string& startNodeId, const std::string& endNodeId,
    const std::unordered_set<std::string>& excludeNodeIds) {
    const auto endIt = nodeIndex_.find(endNodeId);
    if (endIt == nodeIndex_.end() || !adjacency_.count(startNodeId)) {
        return std::nullopt;
    }

    struct OpenNode {
        std::string id;
        double fScore;
        bool operator>(const OpenNode& other) const { return fScore > other.fScore; }
    };

    std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open;
    std::unordered_map<std::string, double> gScore;
    std::unordered_map<std::string, std::string> previous;

    for (const auto& [nodeId, _] : adjacency_) {
        gScore[nodeId] = std::numeric_limits<double>::infinity();
    }
    gScore[startNodeId] = 0;
    open.push({startNodeId, Heuristic(startNodeId, endIt->second)});

    while (!open.empty()) {
        const std::string current = open.top().id;
        open.pop();
        if (current == endNodeId) {
            std::vector<std::string> path;
            std::string cursor = endNodeId;
            while (!cursor.empty()) {
                path.push_back(cursor);
                const auto prevIt = previous.find(cursor);
                cursor = prevIt == previous.end() ? "" : prevIt->second;
            }
            std::reverse(path.begin(), path.end());
            return path.size() > 1 ? std::optional<std::vector<std::string>>(path)
                                   : std::nullopt;
        }

        for (const auto& [neighbor, weight] : adjacency_[current]) {
            if (excludeNodeIds.count(neighbor)) {
                continue;
            }
            const double tentativeG = gScore[current] + weight;
            if (tentativeG >= gScore[neighbor]) {
                continue;
            }
            previous[neighbor] = current;
            gScore[neighbor] = tentativeG;
            open.push({neighbor, tentativeG + Heuristic(neighbor, endIt->second)});
        }
    }
    return std::nullopt;
}

std::optional<std::string> GraphPathfinder::FindNearestNode(const LngLat& position) {
    std::optional<std::string> nearest;
    double minDist = std::numeric_limits<double>::infinity();
    for (const auto& [id, coords] : nodeIndex_) {
        const double dist = std::hypot(coords.x - position.x, coords.y - position.y);
        if (dist < minDist) {
            minDist = dist;
            nearest = id;
        }
    }
    return nearest;
}

std::optional<std::vector<std::string>> GraphPathfinder::PlanCircuit(
    const std::vector<std::string>& nodeIds) {
    if (nodeIds.size() < 2) {
        return nodeIds;
    }
    std::vector<std::string> circuit;
    for (size_t i = 0; i + 1 < nodeIds.size(); ++i) {
        const auto segment = FindPath(nodeIds[i], nodeIds[i + 1]);
        if (!segment) {
            return std::nullopt;
        }
        if (circuit.empty()) {
            circuit = *segment;
        } else {
            circuit.insert(circuit.end(), segment->begin() + 1, segment->end());
        }
    }
    return circuit;
}

std::optional<std::vector<std::string>> GraphPathfinder::PlanGreedyCircuit(
    const std::vector<std::string>& nodeIds) {
    return PlanCircuit(nodeIds);
}

std::optional<std::vector<std::string>> GraphPathfinder::PlanChinesePostmanCircuit() {
    if (graph_.nodes.empty()) {
        return std::vector<std::string>{};
    }
    std::vector<std::string> ids;
    ids.reserve(graph_.nodes.size());
    for (const auto& node : graph_.nodes) {
        ids.push_back(node.id);
    }
    return PlanGreedyCircuit(ids);
}

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy
