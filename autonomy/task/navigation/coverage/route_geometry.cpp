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
 * @file route_geometry.cpp
 * @brief Native complete-coverage route geometry implementation.
 */

#include "autonomy/task/navigation/coverage/route_geometry.hpp"

#include <array>
#include <cmath>
#include <limits>
namespace autonomy::task::coverage {
F2CMultiPoint ReversedConnection(const F2CMultiPoint &mp) {
  std::vector<F2CPoint> pts;
  pts.reserve(mp.size());
  for (int i = static_cast<int>(mp.size()) - 1; i >= 0; --i) {
    pts.emplace_back(mp[i]);
  }
  return F2CMultiPoint(pts);
}

// Reverses group order, each swath, and each connection so a cell can be
// entered from either end.
F2CRoute ReversedRoute(const F2CRoute &route) {
  const auto &groups = route.GetVectorSwaths();
  const auto &conns = route.GetConnections();
  F2CRoute out;
  for (int k = static_cast<int>(groups.size()) - 1; k >= 0; --k) {
    F2CSwaths swaths = groups[k];
    swaths.Reverse();
    for (size_t s = 0; s < swaths.size(); ++s) {
      swaths.at(s).Reverse();
    }
    const size_t conn_after = static_cast<size_t>(k) + 1;
    out.AddConnectedSwaths(conn_after < conns.size()
                               ? ReversedConnection(conns[conn_after])
                               : F2CMultiPoint(),
                           swaths);
  }
  return out;
}

// Fallback matches RouteMode.msg's tsp_d_tol default; the stitch needs a
// positive tolerance.
constexpr double kDefaultDTol = 1e-4;

// Joins consecutive swaths along the cell's border graph. A naked turn ignores
// the boundary, so a SNAKE/SPIRAL skip cuts across already-covered ground.
F2CRoute RouteThroughCell(const Field &cell_geom, const Swaths &ordered,
                          double d_tol, const int &logger) {
  F2CCells cell;
  cell.AddGeometry(cell_geom);
  F2CSwathsByCells cell_swaths;
  cell_swaths.emplace_back(ordered);

  try {
    f2c::rp::RoutePlannerBase rp;
    F2CGraph2D graph = rp.CreateShortestGraph(cell, cell_swaths, d_tol);

    // Per-pair fallback: shortestPath throws on a non-node endpoint, and one
    // such pair must not cost the whole cell its connections.
    F2CRoute route;
    for (size_t i = 0; i < ordered.size(); ++i) {
      if (i > 0) {
        std::vector<F2CPoint> conn;
        try {
          conn =
              graph.ShortestPath(route.EndPoint(), ordered.at(i).StartPoint());
        } catch (const std::exception &) {
          conn.clear();
        }
        route.AddConnection(conn);
      }
      route.AddSwath(ordered.at(i));
    }
    return route;
  } catch (const std::exception &e) {
    AWARN_UNUSED(logger, "Cell border graph failed (%s); using plain turns.",
                 e.what());
    F2CRoute plain;
    plain.AddConnectedSwaths(F2CMultiPoint(), ordered);
    return plain;
  }
}

// Orders per-cell routes nearest-neighbour (heading-aware) and bridges them
// along the travel-cell border graph. Independent of how each cell route was
// planned.
F2CRoute StitchCellRoutes(const F2CCells &travel_cells,
                          const std::vector<F2CRoute> &cell_routes,
                          const std::optional<F2CPoint> &start_end,
                          double d_tol, const int &logger);

F2CRoute StitchCellRoutes(const F2CCells &travel_cells,
                          const std::vector<F2CRoute> &cell_routes,
                          const std::optional<F2CPoint> &start_end,
                          double d_tol, const int &logger) {
  const double tol = d_tol > 0.0 ? d_tol : kDefaultDTol;

  std::vector<size_t> remaining;
  for (size_t i = 0; i < cell_routes.size(); ++i) {
    if (!cell_routes[i].IsEmpty()) {
      remaining.push_back(i);
    }
  }
  if (remaining.empty()) {
    return F2CRoute();
  }

  // Nearest-neighbor cell order; entry cost also penalizes heading mismatch to
  // avoid S-maneuvers.
  const auto angle_difference = [](double a, double b) {
    return std::fabs(std::atan2(std::sin(a - b), std::cos(a - b)));
  };
  const auto pick_nearest = [&cell_routes, &remaining, &angle_difference](
                                const F2CPoint &from,
                                const std::optional<double> &from_angle,
                                bool &reversed) {
    size_t best_idx = remaining[0];
    double best_cost = std::numeric_limits<double>::max();
    for (const size_t idx : remaining) {
      const auto &groups = cell_routes[idx].GetVectorSwaths();
      const F2CSwath &first_swath = groups.front().at(0);
      const F2CSwath &last_swath = groups.back().back();
      const std::array<std::pair<F2CPoint, double>, 2> entries = {
          std::make_pair(cell_routes[idx].StartPoint(),
                         first_swath.GetInAngle()),
          std::make_pair(cell_routes[idx].EndPoint(),
                         last_swath.GetOutAngle() + M_PI)};
      for (size_t e = 0; e < entries.size(); ++e) {
        const double dist = from.Distance(entries[e].first);
        double cost = dist;
        if (from_angle && dist > 1e-6) {
          const double bearing = (entries[e].first - from).GetAngleFromPoint();
          cost += first_swath.GetWidth() *
                  (angle_difference(*from_angle, bearing) +
                   angle_difference(bearing, entries[e].second));
        }
        if (cost < best_cost) {
          best_cost = cost;
          best_idx = idx;
          reversed = (e == 1);
        }
      }
    }
    return best_idx;
  };

  size_t current = remaining[0];
  bool current_reversed = false;
  if (start_end) {
    AWARN_UNUSED(logger,
                 "Multi-cell route: start_pose picks the nearest cell; the "
                 "route starts at "
                 "that cell's own start, not at the exact point.");
    current = pick_nearest(*start_end, std::nullopt, current_reversed);
  }

  // Bridges follow the travel-cell pair's border graph, not a line that could
  // cut a void.
  const auto build_bridge =
      [&travel_cells, tol](const F2CSwath &from_swath, const F2CSwath &to_swath,
                           const F2CPoint &from, const F2CPoint &to) {
        F2CSwaths end_swaths;
        end_swaths.emplace_back(from_swath);
        end_swaths.emplace_back(to_swath);
        F2CSwathsByCells bridge_swaths;
        bridge_swaths.emplace_back(end_swaths);

        const auto via_graph = [&bridge_swaths, tol, &from,
                                &to](const F2CCells &graph_cells) {
          std::vector<F2CPoint> pts;
          try {
            f2c::rp::RoutePlannerBase rp;
            F2CGraph2D graph =
                rp.CreateShortestGraph(graph_cells, bridge_swaths, tol);
            pts = graph.ShortestPath(from, to);
          } catch (const std::exception &) {
            pts.clear();
          }
          return pts;
        };

        F2CCells pair_cells;
        const Field cell_a = travel_cells.GetCellWherePoint(from);
        if (cell_a.size() > 0) {
          pair_cells.AddGeometry(cell_a);
        }
        const Field cell_b = travel_cells.GetCellWherePoint(to);
        if (cell_b.size() > 0) {
          pair_cells.AddGeometry(cell_b);
        }

        // Both endpoints must have landed in a cell: a one-cell graph still
        // returns a plausible path, reaching the far endpoint by jumping off
        // that border.
        std::vector<F2CPoint> bridge;
        if (pair_cells.size() == 2) {
          bridge = via_graph(pair_cells);
        }
        if (bridge.size() < 2) {
          bridge = via_graph(travel_cells);
        }
        if (bridge.size() < 2) {
          bridge = {from, to};
        }
        return bridge;
      };

  F2CRoute merged;
  std::optional<F2CSwath> last_swath;
  while (true) {
    remaining.erase(std::find(remaining.begin(), remaining.end(), current));
    const F2CRoute cell_route = current_reversed
                                    ? ReversedRoute(cell_routes[current])
                                    : cell_routes[current];
    if (!merged.IsEmpty() && last_swath) {
      merged.AddConnection(
          build_bridge(*last_swath, cell_route.GetVectorSwaths().front().at(0),
                       merged.EndPoint(), cell_route.StartPoint()));
    }
    const auto &vec_swaths = cell_route.GetVectorSwaths();
    const auto &connections = cell_route.GetConnections();
    for (size_t k = 0; k < vec_swaths.size(); ++k) {
      merged.AddConnectedSwaths(
          k < connections.size() ? connections[k] : F2CMultiPoint(),
          vec_swaths[k]);
    }
    last_swath = cell_route.GetVectorSwaths().back().back();

    if (remaining.empty()) {
      break;
    }
    current = pick_nearest(
        merged.EndPoint(),
        last_swath ? std::optional<double>(last_swath->GetOutAngle())
                   : std::nullopt,
        current_reversed);
  }
  return merged;
}

}  // namespace autonomy::task::coverage
