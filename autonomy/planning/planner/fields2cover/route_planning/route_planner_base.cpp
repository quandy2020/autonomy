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
 * @file route_planner_base.cpp
 * @brief Fields2Cover route planner base implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>
#include "autonomy/planning/planner/fields2cover/route_planning/route_planner_base.hpp"

namespace autonomy::planning::planner::fields2cover::rp {

F2CRoute RoutePlannerBase::GenRoute(
    const F2CCells& cells, const F2CSwathsByCells& swaths,
    bool show_log, double d_tol, bool redirect_swaths,
    long int time_limit_seconds, bool search_for_optimum) {
  F2CGraph2D shortest_graph = CreateShortestGraph(cells, swaths, d_tol);

  F2CGraph2D cov_graph = CreateCoverageGraph(
      cells, swaths, shortest_graph, d_tol, redirect_swaths);

  std::vector<long long int> v_route = ComputeBestRoute(
      cov_graph, show_log, time_limit_seconds, search_for_optimum);
  return TransformSolutionToRoute(
      v_route, swaths, cov_graph, shortest_graph);
}

void RoutePlannerBase::SetStartAndEndPoint(const F2CPoint& p) {
  this->r_start_end = p;
}

F2CGraph2D RoutePlannerBase::CreateShortestGraph(
    const F2CCells& cells, const F2CSwathsByCells& swaths_by_cells,
    double d_tol) const {
  F2CGraph2D g;
  // Connect each swath end to the border of the cell it belongs to. An end
  // outside every cell is left out: a line to the nearest border would cross
  // whatever lies between.
  auto AddBorderEdge = [&g, &cells, d_tol] (const F2CPoint& p) {
    F2CCell cell = cells.GetCellWherePoint(p, d_tol);
    if (!cell.IsEmpty()) {
      g.AddEdge(p, cell.ClosestPointOnBorderTo(p));
    }
  };
  for (auto&& swaths : swaths_by_cells) {
    for (auto&& s : swaths) {
      AddBorderEdge(s.StartPoint());
      AddBorderEdge(s.EndPoint());
    }
  }

  // Add points in the border
  for (auto&& cell : cells) {
    for (auto&& ring : cell) {
      for (size_t i = 0; i < ring.size()-1; ++i) {
        g.AddEdge(ring.GetGeometry(i), ring.GetGeometry(i+1));
      }
    }
  }

  // Add start and end point if they exists. Unlike a swath end, this point
  // is allowed to sit outside the field, so it always gets its border edge.
  if (this->r_start_end) {
    g.AddEdge(*r_start_end, cells.ClosestPointOnBorderTo(*r_start_end));
  }

  std::vector<F2CPoint> nodes = g.GetNodes();

  // Connect nodes that are near other edges.
  for (int i = 0; i < 2; ++i) {
    auto edges = g.GetEdges();
    for (auto&& edge : edges) {
      size_t from = edge.first;
      for (auto&& e : edge.second) {
        size_t to = e.first;

        F2CLineString border {g.IndexToNode(from), g.IndexToNode(to)};
        for (auto&& n : nodes) {
          if (n != border[0] && n != border[1] && n.Distance(border) < d_tol) {
            g.AddEdge(border[0], n);
            g.AddEdge(n, border[1]);
            g.RemoveEdge(border[0], border[1]);
          }
        }
      }
    }
  }
  return g;
}


F2CGraph2D RoutePlannerBase::CreateCoverageGraph(
    const F2CCells& cells, const F2CSwathsByCells& swaths_by_cells,
    F2CGraph2D& shortest_graph,
    double d_tol, bool redirect_swaths) const {
  F2CGraph2D g;
  for (auto&& swaths : swaths_by_cells) {
    for (auto&& s : swaths) {
      F2CPoint mid_p {(s.StartPoint() + s.EndPoint()) * 0.5};
      if (redirect_swaths) {
        g.AddEdge(s.StartPoint(), mid_p, 0);
        g.AddEdge(s.EndPoint(), mid_p, 0);
      } else {
        g.AddDirectedEdge(s.StartPoint(), mid_p, 0);
        g.AddDirectedEdge(mid_p, s.EndPoint(), 0);
      }
    }
  }

  for (const auto& swaths1 : swaths_by_cells) {
    for (const auto& s1 : swaths1) {
      auto s1_s = s1.StartPoint();
      auto s1_e = s1.EndPoint();
      for (const auto& swaths2 : swaths_by_cells) {
        for (const auto& s2 : swaths2) {
          auto s2_s = s2.StartPoint();
          auto s2_e = s2.EndPoint();
          if (redirect_swaths) {
            g.AddEdge(s1_s, s2_s, shortest_graph);
            g.AddEdge(s1_e, s2_e, shortest_graph);
            g.AddEdge(s1_s, s2_e, shortest_graph);
            g.AddEdge(s1_e, s2_s, shortest_graph);
          } else {
            g.AddDirectedEdge(s1_e, s2_s, shortest_graph);
          }
        }
      }
    }
  }

  F2CPoint deposit(-1e8, -1e8);  // Arbitrary point
  if (this->r_start_end) {
    deposit = *this->r_start_end;
  }

  for (auto&& swaths : swaths_by_cells) {
    for (auto&& s : swaths) {
      if (this->r_start_end) {
        g.AddEdge(s.StartPoint(), deposit, shortest_graph);
        g.AddEdge(s.EndPoint(), deposit, shortest_graph);
      } else {
        g.AddEdge(s.StartPoint(), deposit, 0);
        g.AddEdge(s.EndPoint(), deposit, 0);
      }
    }
  }
  return g;
}

std::vector<long long int> RoutePlannerBase::ComputeBestRoute(
    const F2CGraph2D& cov_graph, bool show_log, long int time_limit_seconds,
    bool use_guided_local_search) const {
  (void)show_log;
  (void)use_guided_local_search;
  const int n = static_cast<int>(cov_graph.NumNodes());
  if (n <= 1) {
    return {};
  }
  const int depot = n - 1;
  auto cost = [&cov_graph](int from, int to) -> int64_t {
    return cov_graph.GetCostFromEdge(static_cast<size_t>(from),
                                     static_cast<size_t>(to));
  };

  std::vector<int> tour;
  tour.reserve(static_cast<size_t>(n - 1));
  std::vector<char> visited(static_cast<size_t>(n), 0);
  visited[static_cast<size_t>(depot)] = 1;
  int current = depot;
  for (int step = 0; step < n - 1; ++step) {
    int best = -1;
    int64_t best_cost = std::numeric_limits<int64_t>::max();
    for (int cand = 0; cand < n; ++cand) {
      if (visited[static_cast<size_t>(cand)]) {
        continue;
      }
      const int64_t c = cost(current, cand);
      if (best < 0 || c < best_cost) {
        best = cand;
        best_cost = c;
      }
    }
    if (best < 0) {
      break;
    }
    visited[static_cast<size_t>(best)] = 1;
    tour.push_back(best);
    current = best;
  }

  auto tour_cost = [&](const std::vector<int>& t) {
    int64_t total = cost(depot, t.front());
    for (size_t i = 0; i + 1 < t.size(); ++i) {
      total += cost(t[i], t[i + 1]);
    }
    total += cost(t.back(), depot);
    return total;
  };

  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(std::max<long int>(time_limit_seconds, 0));
  bool improved = true;
  while (improved && std::chrono::steady_clock::now() < deadline) {
    improved = false;
    for (size_t i = 0; i + 1 < tour.size(); ++i) {
      for (size_t k = i + 1; k < tour.size(); ++k) {
        std::vector<int> candidate = tour;
        std::reverse(candidate.begin() + static_cast<std::ptrdiff_t>(i),
                     candidate.begin() + static_cast<std::ptrdiff_t>(k + 1));
        if (tour_cost(candidate) < tour_cost(tour)) {
          tour.swap(candidate);
          improved = true;
        }
      }
    }
  }

  std::vector<long long int> v_id;
  v_id.reserve(tour.size());
  for (int id : tour) {
    v_id.push_back(id);
  }
  return v_id;
}

F2CRoute RoutePlannerBase::TransformSolutionToRoute(
    const std::vector<long long int>& route_ids,
    const F2CSwathsByCells& swaths_by_cells,
    const F2CGraph2D& coverage_graph,
    F2CGraph2D& shortest_graph) const {
  F2CRoute route;
  const size_t NS = swaths_by_cells.SizeTotal();
  for (int i = 0; i < route_ids.size()-2; ++i) {
    F2CPoint p_s = coverage_graph.IndexToNode(route_ids[i]);
    F2CPoint p_e = coverage_graph.IndexToNode(route_ids[i+2]);
    for (int j = 0; j < NS; ++j) {
      F2CSwath swath = swaths_by_cells.GetSwath(j).Clone();
      if (p_s == swath.StartPoint() && p_e == swath.EndPoint()) {
        if (route.IsEmpty() && r_start_end) {
          route.AddConnection(shortest_graph.ShortestPath(
                *r_start_end, swath.StartPoint()));
        }
        route.AddSwath(swath, shortest_graph);
        break;
      } else if (p_e == swath.StartPoint() && p_s == swath.EndPoint()) {
        swath.Reverse();
        if (route.IsEmpty() && r_start_end) {
          route.AddConnection(shortest_graph.ShortestPath(
                *r_start_end, swath.StartPoint()));
        }
        route.AddSwath(swath, shortest_graph);
        break;
      }
    }
  }
  if (r_start_end) {
    route.AddConnection(shortest_graph.ShortestPath(
          route.EndPoint(), *r_start_end));
  }
  return route;
}

}  // namespace autonomy::planning::planner::fields2cover::rp


