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

/**
 * @file graph2d.cpp
 * @brief Fields2Cover Graph2D implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <numeric>
#include "autonomy/planning/planner/fields2cover/types/graph2d.hpp"

namespace autonomy::planning::planner::fields2cover::types {

Graph2D& Graph2D::AddDirectedEdge(
    const Point& from, const Point& to, int64_t cost) {
  size_t p_from = this->nodes_to_index_.insert(
      std::make_pair(from, nodes_to_index_.size())).first->second;
  size_t p_to   = this->nodes_to_index_.insert(
      std::make_pair(to,   nodes_to_index_.size())).first->second;
  this->index_to_nodes_.insert({{p_from, from}, {p_to, to}});
  this->AddDirectedEdge(p_from, p_to, cost);
  return *this;
}

Graph2D& Graph2D::AddEdge(const Point& i, const Point& j, int64_t cost) {
  this->AddDirectedEdge(i, j, cost);
  return this->AddDirectedEdge(j, i, cost);
}

Graph2D& Graph2D::AddDirectedEdge(const Point& from, const Point& to) {
  return AddDirectedEdge(from, to, int64_t(scale_ * from.Distance(to)));
}

Graph2D& Graph2D::AddEdge(const Point& i, const Point& j) {
  return AddEdge(i, j, int64_t(scale_ * i.Distance(j)));
}

Graph2D& Graph2D::AddDirectedEdge(
    const Point& from, const Point& to, Graph2D& short_path_g) {
  return AddDirectedEdge(from, to, short_path_g.ShortestPathCost(from, to));
}

Graph2D& Graph2D::AddEdge(
    const Point& i, const Point& j, Graph2D& short_path_g) {
  AddDirectedEdge(i, j, short_path_g.ShortestPathCost(i, j));
  return AddDirectedEdge(j, i, short_path_g.ShortestPathCost(j, i));
}


Graph2D& Graph2D::RemoveDirectedEdge(const Point& from, const Point& to) {
  if (!HasNode(from) || !HasNode(to)) {
    return *this;
  }
  this->RemoveDirectedEdge(NodeToIndex(from), NodeToIndex(to));
  return *this;
}

Graph2D& Graph2D::RemoveEdge(const Point& i, const Point& j) {
  return this->RemoveDirectedEdge(i, j).RemoveDirectedEdge(j, i);
}

size_t Graph2D::NumNodes() const {
  return this->nodes_to_index_.size();
}

std::vector<Point> Graph2D::GetNodes() const {
  std::vector<Point> nodes;
  for (auto&& n : nodes_to_index_) {
    nodes.emplace_back(n.first);
  }
  return nodes;
}

bool Graph2D::HasNode(const Point& p) const {
  return this->nodes_to_index_.count(p) > 0;
}

size_t Graph2D::NodeToIndex(const Point& p) const {
  return this->nodes_to_index_.at(p);
}

Point Graph2D::IndexToNode(size_t id) const {
  return this->index_to_nodes_.at(id);
}

std::vector<std::vector<Point>> Graph2D::AllPathsBetween(
    const Point& from, const Point& to) const {
  if (!HasNode(from) || !HasNode(to)) {
    return {};
  }
  auto int_routes = this->AllPathsBetween(NodeToIndex(from), NodeToIndex(to));

  std::vector<std::vector<Point>> routes;
  for (auto&& i_route : int_routes) {
    routes.emplace_back();
    for (auto&& i : i_route) {
      routes.back().emplace_back(IndexToNode(i));
    }
  }
  return routes;
}

std::vector<Point> Graph2D::ShortestPath(
    const Point& from, const Point& to, int64_t INF) {
  if (!HasNode(from) || !HasNode(to)) {
    return {};
  }
  auto i_path = this->ShortestPath(NodeToIndex(from), NodeToIndex(to), INF);
  std::vector<Point> path;
  for (auto&& i : i_path) {
    path.emplace_back(IndexToNode(i));
  }
  return path;
}

int64_t Graph2D::ShortestPathCost(
    const Point& from, const Point& to, int64_t INF) {
  if (!HasNode(from) || !HasNode(to)) {
    return INF;
  }
  return this->ShortestPathCost(NodeToIndex(from), NodeToIndex(to), INF);
}


}  // namespace autonomy::planning::planner::fields2cover::types


