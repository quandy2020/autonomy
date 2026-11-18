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
 * @file graph2d.hpp
 * @brief Fields2Cover Graph2D.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include <unordered_map>
#include "autonomy/planning/planner/fields2cover/types/graph.hpp"
#include "autonomy/planning/planner/fields2cover/types/point.hpp"

namespace autonomy::planning::planner::fields2cover::types {

class Graph2D : public Graph {
 public:
  using Graph::AddDirectedEdge;
  using Graph::AddEdge;
  using Graph::RemoveDirectedEdge;
  using Graph::RemoveEdge;
  using Graph::AllPathsBetween;
  using Graph::ShortestPath;
  using Graph::ShortestPathCost;

  Graph2D& AddDirectedEdge(const Point& from, const Point& to, int64_t cost);
  Graph2D& AddEdge(const Point& i, const Point& j, int64_t cost);
  Graph2D& AddDirectedEdge(const Point& from, const Point& to);
  Graph2D& AddEdge(const Point& i, const Point& j);
  Graph2D& AddDirectedEdge(
      const Point& from, const Point& to, Graph2D& short_path_g);
  Graph2D& AddEdge(
      const Point& i, const Point& j, Graph2D& short_path_g);

  Graph2D& RemoveDirectedEdge(const Point& from, const Point& to);
  Graph2D& RemoveEdge(const Point& i, const Point& j);

  size_t NumNodes() const;
  std::vector<Point> GetNodes() const;

  /**
   * @brief Check if a point is a node of this graph.
   */
  bool HasNode(const Point& p) const;

  size_t NodeToIndex(const Point& p) const;
  Point IndexToNode(size_t id) const;

  /**
   * @brief Paths between two nodes. Empty if either point is not a node.
   */
  std::vector<std::vector<Point>> AllPathsBetween(
      const Point& from, const Point& to) const;

  /**
   * @brief Shortest path between two nodes.
   * Empty if there is no path or either point is not a node.
   */
  std::vector<Point> ShortestPath(const Point& from, const Point& to,
        int64_t INF = 1<<30);

  /**
   * @brief Cost of the shortest path between two nodes.
   * INF if there is no path or either point is not a node.
   */
  int64_t ShortestPathCost(const Point& from, const Point& to,
        int64_t INF = 1<<30);

 protected:
  std::unordered_map<Point, size_t> nodes_to_index_;
  std::unordered_map<size_t, Point> index_to_nodes_;

 private:
  double scale_ {1e3};
};

}  // namespace autonomy::planning::planner::fields2cover::types

