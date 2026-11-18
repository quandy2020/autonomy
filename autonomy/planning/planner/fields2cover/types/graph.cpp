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
 * @file graph.cpp
 * @brief Fields2Cover Graph implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <numeric>
#include <algorithm>
#include "autonomy/planning/planner/fields2cover/types/graph.hpp"

namespace autonomy::planning::planner::fields2cover::types {

Graph& Graph::AddDirectedEdge(size_t from, size_t to, int64_t cost) {
  this->edges_[from][to] =  cost;
  if (!this->next_.empty() || !this->distance_.empty()) {
    this->next_.clear();
    this->distance_.clear();
    this->shortest_paths_.clear();
  }
  return *this;
}

Graph& Graph::AddEdge(size_t i, size_t j, int64_t cost) {
  return this->AddDirectedEdge(i, j, cost).AddDirectedEdge(j, i, cost);
}


Graph& Graph::RemoveDirectedEdge(size_t from, size_t to) {
  if (this->edges_.count(from) == 0 || this->edges_.at(from).count(to) == 0) {
    return *this;
  }
  this->edges_.at(from).erase(to);
  return *this;
}
Graph& Graph::RemoveEdge(size_t i, size_t j) {
  return this->RemoveDirectedEdge(i, j).RemoveDirectedEdge(j, i);
}

std::unordered_set<size_t> Graph::GetNodes() const {
  std::unordered_set<size_t> nodes;
  for (const auto& from : this->edges_) {
    nodes.insert(from.first);
    for (const auto& to : from.second) {
      nodes.insert(to.first);
    }
  }
  return nodes;
}

size_t Graph::NumNodes() const {
  return this->GetNodes().size();
}

size_t Graph::MatrixSize() const {
  size_t size = 0;
  for (const auto& from : this->edges_) {
    size = std::max(size, from.first + 1);
    for (const auto& to : from.second) {
      size = std::max(size, to.first + 1);
    }
  }
  return size;
}

size_t Graph::NumEdges() const {
  return std::accumulate(this->edges_.begin(), this->edges_.end(), 0,
      [](int sum, const auto& e) {return sum + e.second.size();});
}

map_to_map_to_int Graph::GetEdges() const {
  return this->edges_;
}

std::vector<size_t> Graph::GetEdgesFrom(size_t s) const {
  if (this->edges_.count(s) == 0) {
    return {};
  }
  std::vector<size_t> connections;
  for (auto&& i : this->edges_.at(s)) {
    connections.emplace_back(i.first);
  }
  return connections;
}

int64_t Graph::GetCostFromEdge(size_t from, size_t to, int64_t INF) const {
  if (this->edges_.count(from) == 0 || this->edges_.at(from).count(to) == 0) {
    return INF;
  }
  return this->edges_.at(from).at(to);
}

std::vector<std::vector<size_t>> Graph::AllPathsBetween(
    size_t from, size_t to) const {
  std::vector<bool> visited(this->MatrixSize(), false);
  std::vector<std::vector<size_t>> routes(1);
  int route_index = 0;
  this->DFS(from, to, routes, visited, route_index);
  routes.erase(std::remove_if(routes.begin(), routes.end(),
      [&to] (const std::vector<size_t> x) {
          return (x.size() < 1 || x.back() != to);
      }), routes.end());
  return routes;
}

void Graph::DFS(
    size_t from, size_t to,
    std::vector<std::vector<size_t>>& routes,
    std::vector<bool>& visited,
    int& route_index) const {
  routes.emplace_back(routes[route_index]);
  routes.back().emplace_back(from);
  int i_route = routes.size() - 1;
  if (from != to) {
    visited[from] = true;
    for (auto&& i : this->GetEdgesFrom(from)) {
      if (!visited[i]) {
        this->DFS(i, to, routes, visited, i_route);
      }
    }
    visited[from] = false;
  }
}

void  Graph::ShortestPathsAndCosts(int64_t INF) {
  const size_t N = this->MatrixSize();
  this->distance_.assign(N, std::vector<int64_t>(N, INF));
  this->next_.assign(N, std::vector<int64_t>(N, -1));
  InitializeMatrices(distance_,next_, INF);
}


void Graph::InitializeMatrices(std::vector<std::vector<int64_t>>& dist,
    std::vector<std::vector<int64_t>>& next,
    int64_t INF) {

  const size_t N = this->MatrixSize();

  // Initialize distances and paths for direct connections
  for (const auto& src : this->edges_) {
    for (const auto& dest : src.second) {
      dist[src.first][dest.first] = dest.second;
      next[src.first][dest.first] = dest.first;
    }
    dist[src.first][src.first] = 0;
  }


  // Floyd-Warshall
  for (size_t k = 0; k < N; ++k) {
    for (size_t i = 0; i < N; ++i) {
      for (size_t j = 0; j < N; ++j) {
        if (dist[i][k] < INF && \
            dist[k][j] < INF && \
            dist[i][j] > dist[i][k] + dist[k][j]) {
            dist[i][j] = dist[i][k] + dist[k][j];
            next[i][j] = next[i][k];
        }
      }
    }
  }

  // Reconstruct paths
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = 0; j < N; ++j) {
      if (i != j && next[i][j] != -1) {
        size_t current = i;
        while (current != j) {
          current = next[current][j];
        }
      } else if (i != j && next[i][j] == -1) {
        dist[i][j] = INF;
      }
    }
  }
}

std::vector<std::vector<int64_t>>& Graph::GetCosts() {
  if (this->distance_.empty()) {
    this->ShortestPathsAndCosts();
  }
  return this->distance_;
}

short_path_container_t& Graph::GetPaths() {
  if (next_.empty()) {
    this->ShortestPathsAndCosts();
  }
  if (shortest_paths_.empty()) {
    const size_t N = this->MatrixSize();
    shortest_paths_.assign(N, std::vector<std::vector<size_t>>(N));
    for (size_t i = 0; i < N; ++i) {
      for (size_t j = 0; j < N; ++j) {
        if (i != j && next_[i][j] != -1) {
          std::vector<size_t> path = {i};
          size_t current = i;
          while (current != j) {
            current = next_[current][j];
            path.push_back(current);
          }
          shortest_paths_[i][j] = path;
        }
      }
    }
  }
  return this->shortest_paths_;
}

std::vector<size_t> Graph::ShortestPath(size_t from, size_t to, int64_t INF) {
  if (!this->edges_.empty() && (this->next_.empty() || this->distance_.empty())) {
    this->ShortestPathsAndCosts(INF);
  }
  return this->ReconstructPath(from, to, next_);
}

std::vector<size_t> Graph::ReconstructPath(size_t from,
    size_t to, std::vector<std::vector<int64_t>>& next) const {
  if (from != to && next[from][to] != -1) {
    std::vector<size_t> path = {from};
    size_t current = from;
    while (current != to) {
      current = next[current][to];
      path.push_back(current);
    }
    return path;
  } else {
    return {};
  }
}

int64_t Graph::ShortestPathCost(size_t from, size_t to, int64_t INF) {
  if (!this->edges_.empty() && this->distance_.size() == 0) {
    this->ShortestPathsAndCosts(INF);
  }
  return this->distance_[from][to];
}

}  // namespace autonomy::planning::planner::fields2cover::types


