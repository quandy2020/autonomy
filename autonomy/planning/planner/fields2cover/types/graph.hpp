/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file graph.hpp
 * @brief Fields2Cover Graph.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <cstdint>
#include <vector>
#include <unordered_set>
#include <functional>
#include <utility>
#include <unordered_map>

namespace autonomy::planning::planner::fields2cover::types {

// map.at(from).at(to) = cost(from->to)
typedef std::unordered_map<size_t, std::unordered_map<size_t, int64_t>>
  map_to_map_to_int;

// shortest_path[from][to] = {(size_t) node_from, node_1, node_2 ,...,node_to}
typedef std::vector<std::vector<std::vector<size_t>>>  short_path_container_t;

class Graph {
 public:
  Graph& AddDirectedEdge(size_t from, size_t to, int64_t cost);
  Graph& AddEdge(size_t i, size_t j, int64_t cost);
  Graph& RemoveDirectedEdge(size_t from, size_t to);
  Graph& RemoveEdge(size_t i, size_t j);

  /**
   * @brief Ids of every node, including nodes that only have incoming edges.
   */
  std::unordered_set<size_t> GetNodes() const;

  size_t NumNodes() const;

  /**
   * @brief Size of the matrices returned by GetCosts() and GetPaths().
   * Nodes are indexed by their own id, so this is the largest id plus one.
   */
  size_t MatrixSize() const;

  size_t NumEdges() const;
  map_to_map_to_int GetEdges() const;

  std::vector<size_t> GetEdgesFrom(size_t s) const;

  int64_t GetCostFromEdge(size_t from, size_t to,
      int64_t INF = 1e15) const;

  std::vector<std::vector<size_t>> AllPathsBetween(
      size_t from, size_t to) const;

  void ShortestPathsAndCosts(int64_t INF = 1e15);

  void InitializeMatrices(std::vector<std::vector<int64_t>>& cost_dest,
      std::vector<std::vector<int64_t>>& next_dest,
      int64_t INF = 1e15);

  std::vector<std::vector<int64_t>>& GetCosts();

  short_path_container_t& GetPaths();

  std::vector<size_t> ShortestPath(size_t from, size_t to,
        int64_t INF = 1e15);

  std::vector<size_t> ReconstructPath(size_t from,
      size_t to,
      std::vector<std::vector<int64_t>>& next) const;

  int64_t ShortestPathCost(size_t from, size_t to,
        int64_t INF = 1e15);

 protected:
  void DFS(size_t from, size_t to,
    std::vector<std::vector<size_t>>& routes,
    std::vector<bool>& visited,
    int& route_index) const;

 protected:
  map_to_map_to_int edges_;
  short_path_container_t shortest_paths_;       // sequence of nodes
  std::vector<std::vector<int64_t>> distance_;  // costs of shortest paths
  std::vector<std::vector<int64_t>> next_;
};

}  // namespace autonomy::planning::planner::fields2cover::types

