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

/**
 * @file path_searcher.cpp
 * @brief A* and jump-point search plus line-of-sight and heat-aware path cleanup.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/path_searcher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {
namespace {

/**
 * @brief Open-set entry ordered by f = g + heuristic.
 */
struct Node {
  double f{0.0};  ///< Estimated total cost, meters plus weighted extras.
  int index{0};   ///< Row-major cell index.
};

/**
 * @brief True when the cell center lies inside the axis-aligned search window.
 * @param grid Grid that converts the index to world coordinates.
 * @param column_index Column.
 * @param row_index Row.
 * @param weights Window limits in world meters.
 */
bool IsInsideWindow(const OccupancyGrid& grid, int column_index, int row_index, const SearchWeights& weights) {
  double x = 0.0;
  double y = 0.0;
  grid.ConvertCellToWorld(column_index, row_index, &x, &y);
  return x >= weights.window_min_x && x <= weights.window_max_x && y >= weights.window_min_y &&
         y <= weights.window_max_y;
}

/**
 * @brief True when the cell is outside the window or occupied and not soft.
 * @param grid Occupancy.
 * @param column_index Column.
 * @param row_index Row.
 * @param weights allow_soft_occupied turns an occupied cell into a costly cell instead of a blocked one.
 */
bool IsCellBlocked(const OccupancyGrid& grid, int column_index, int row_index, const SearchWeights& weights) {
  if (!IsInsideWindow(grid, column_index, row_index, weights)) {
    return true;
  }
  const Cell cell = grid.GetCell(column_index, row_index);
  return cell == Cell::kOccupied && !weights.allow_soft_occupied;
}

/**
 * @brief True when a jump may land on the cell.
 *
 * The cell must be inside the window and labeled free. Unknown cells stop a
 * jump even when the A* unknown weight would have allowed them.
 *
 * @param grid Occupancy.
 * @param column_index Column.
 * @param row_index Row.
 * @param weights Window. The cost weights are not used.
 */
bool IsJumpCellPassable(const OccupancyGrid& grid, int column_index, int row_index, const SearchWeights& weights) {
  if (column_index < 0 || row_index < 0 || column_index >= grid.GetWidth() || row_index >= grid.GetHeight()) {
    return false;
  }
  if (!IsInsideWindow(grid, column_index, row_index, weights)) {
    return false;
  }
  return grid.GetCell(column_index, row_index) == Cell::kFree;
}

/**
 * @brief Unit direction hint, flipped when it points away from the goal.
 *
 * A zero hint stays zero and disables the alignment cost. Otherwise the hint
 * is normalized and negated when its dot product with (goal - start) is negative.
 *
 * @param weights reference_direction_x and reference_direction_y form the hint.
 * @param start_x Start x, meters.
 * @param start_y Start y, meters.
 * @param goal_x Goal x, meters.
 * @param goal_y Goal y, meters.
 * @param reference_x Output unit x. Required.
 * @param reference_y Output unit y. Required.
 */
void ComputeReferenceDirection(const SearchWeights& weights, double start_x, double start_y, double goal_x, double goal_y,
                        double* rx, double* ry) {
  double direction_x = weights.reference_direction_x;
  double direction_y = weights.reference_direction_y;
  const double goal_offset_x = goal_x - start_x;
  const double goal_offset_y = goal_y - start_y;
  if (direction_x * direction_x + direction_y * direction_y < 1e-12) {
    direction_x = goal_offset_x;
    direction_y = goal_offset_y;
  } else if (direction_x * goal_offset_x + direction_y * goal_offset_y < 0.0) {
    direction_x = -direction_x;
    direction_y = -direction_y;
  }
  *rx = direction_x;
  *ry = direction_y;
}

/**
 * @brief Walk parent pointers from the goal back to the start and emit cell centers.
 *
 * The polyline is reversed so it begins at the start. The exact start and goal
 * world coordinates replace the first and last cell centers.
 *
 * @param grid Converts cell indices to world coordinates.
 * @param parent Parent index of each cell, or -1 at the start.
 * @param start Row-major index of the start cell.
 * @param goal Row-major index of the goal cell.
 * @param start_x Exact start x, meters.
 * @param start_y Exact start y, meters.
 * @param goal_x Exact goal x, meters.
 * @param goal_y Exact goal y, meters.
 * @param path Output polyline. Required.
 * @return False when the parent chain does not reach the start.
 */
bool ReconstructSearchPath(const OccupancyGrid& grid, const std::vector<int>& parent, int start, int goal,
                 double start_x, double start_y, double goal_x, double goal_y, std::vector<Eigen::Vector2d>* path) {
  std::vector<int> rev;
  for (int i = goal; i >= 0; i = parent[static_cast<size_t>(i)]) {
    rev.push_back(i);
    if (i == start) {
      break;
    }
    if (parent[static_cast<size_t>(i)] < 0 && i != start) {
      return false;
    }
  }
  if (rev.empty() || rev.back() != start) {
    return false;
  }
  path->clear();
  path->reserve(rev.size());
  for (int i = static_cast<int>(rev.size()) - 1; i >= 0; --i) {
    const int column_index = rev[static_cast<size_t>(i)] % grid.GetWidth();
    const int row_index = rev[static_cast<size_t>(i)] / grid.GetWidth();
    double x = 0.0;
    double y = 0.0;
    grid.ConvertCellToWorld(column_index, row_index, &x, &y);
    if (!path->empty() && (path->back() - Eigen::Vector2d(x, y)).norm() < 1e-4) {
      continue;
    }
    path->emplace_back(x, y);
  }
  if (!path->empty()) {
    path->front() = Eigen::Vector2d(start_x, start_y);
    path->back() = Eigen::Vector2d(goal_x, goal_y);
  }
  return path->size() >= 2;
}

/**
 * @brief True when a capsule from a to b contains no occupied cell.
 *
 * Samples are spaced by half a cell. The capsule radius is line_of_sight_radius_cells.
 * A segment shorter than 0.1 mm is treated as clear.
 *
 * @param grid Occupancy.
 * @param a Segment start, world meters.
 * @param b Segment end, world meters.
 * @param line_of_sight_radius_cells Capsule radius in cells. Zero tests the center line only.
 */
bool HasLineOfSight(const OccupancyGrid& grid, const Eigen::Vector2d& a, const Eigen::Vector2d& b,
               int line_of_sight_radius_cells) {
  const double len = (b - a).norm();
  if (len < 1e-4) {
    return true;
  }
  const Eigen::Vector2d dir = (b - a) / len;
  const double step = std::max(0.5 * grid.GetResolution(), 0.05);
  const int radial = std::max(0, line_of_sight_radius_cells);
  for (double s = 0.0; s <= len; s += step) {
    const Eigen::Vector2d c = a + s * dir;
    for (int row_index = -radial; row_index <= radial; ++row_index) {
      for (int column_index = -radial; column_index <= radial; ++column_index) {
        if (column_index * column_index + row_index * row_index > radial * radial) {
          continue;
        }
        int mx = 0;
        int my = 0;
        const double x = c.x() + column_index * grid.GetResolution();
        const double y = c.y() + row_index * grid.GetResolution();
        if (!grid.ConvertWorldToCell(x, y, &mx, &my) || grid.GetCell(mx, my) == Cell::kOccupied) {
          return false;
        }
      }
    }
  }
  return true;
}

/**
 * @brief Walk from (x, y) along (dx, dy) until a jump point or a blockage.
 *
 * A jump point is the goal, or a cell with a forced neighbor on the side of
 * a diagonal or cardinal step. The walk stops at the first blocked cell and
 * returns false. The step count is capped by the grid width plus height.
 *
 * @param grid Occupancy and dimensions.
 * @param weights Window used by the passable test.
 * @param x Start column. The first tested cell is one step away.
 * @param y Start row.
 * @param dx Step column, -1, 0, or 1.
 * @param dy Step row, -1, 0, or 1.
 * @param goal_x Goal column.
 * @param goal_y Goal row.
 * @param nx Output jump column. Required.
 * @param ny Output jump row. Required.
 * @return False when the ray leaves the free set before a jump point.
 */
bool FindJumpPoint(const OccupancyGrid& grid, const SearchWeights& weights, int x, int y, int dx, int dy,
          int goal_x, int goal_y, int* nx, int* ny) {
  const int limit = grid.GetWidth() + grid.GetHeight();
  int cx = x;
  int cy = y;
  for (int step = 0; step < limit; ++step) {
    cx += dx;
    cy += dy;
    if (!IsJumpCellPassable(grid, cx, cy, weights)) {
      return false;
    }
    if (cx == goal_x && cy == goal_y) {
      *nx = cx;
      *ny = cy;
      return true;
    }
    const bool diagonal = dx != 0 && dy != 0;
    if (!diagonal && dx != 0) {
      if ((!IsJumpCellPassable(grid, cx, cy + 1, weights) && IsJumpCellPassable(grid, cx + dx, cy + 1, weights)) ||
          (!IsJumpCellPassable(grid, cx, cy - 1, weights) && IsJumpCellPassable(grid, cx + dx, cy - 1, weights))) {
        *nx = cx;
        *ny = cy;
        return true;
      }
    } else if (!diagonal && dy != 0) {
      if ((!IsJumpCellPassable(grid, cx + 1, cy, weights) && IsJumpCellPassable(grid, cx + 1, cy + dy, weights)) ||
          (!IsJumpCellPassable(grid, cx - 1, cy, weights) && IsJumpCellPassable(grid, cx - 1, cy + dy, weights))) {
        *nx = cx;
        *ny = cy;
        return true;
      }
    } else {
      if ((!IsJumpCellPassable(grid, cx - dx, cy, weights) &&
           IsJumpCellPassable(grid, cx - dx, cy + dy, weights)) ||
          (!IsJumpCellPassable(grid, cx, cy - dy, weights) &&
           IsJumpCellPassable(grid, cx + dx, cy - dy, weights))) {
        *nx = cx;
        *ny = cy;
        return true;
      }
      int jump_column = 0;
      int jump_row = 0;
      if (FindJumpPoint(grid, weights, cx, cy, dx, 0, goal_x, goal_y, &jump_column, &jump_row) ||
          FindJumpPoint(grid, weights, cx, cy, 0, dy, goal_x, goal_y, &jump_column, &jump_row)) {
        *nx = cx;
        *ny = cy;
        return true;
      }
    }
  }
  return false;
}

}  // namespace

bool PathSearcher::Search(const OccupancyGrid& grid, double start_x, double start_y, double goal_x, double goal_y,
                const SearchWeights& weights, std::vector<Eigen::Vector2d>* path) const {
  path->clear();
  int start_column = 0;
  int start_row = 0;
  int goal_column = 0;
  int goal_row = 0;
  if (!grid.ConvertWorldToCell(start_x, start_y, &start_column, &start_row) || !grid.ConvertWorldToCell(goal_x, goal_y, &goal_column, &goal_row)) {
    return false;
  }
  if (IsCellBlocked(grid, start_column, start_row, weights) || IsCellBlocked(grid, goal_column, goal_row, weights)) {
    return false;
  }

  double reference_x = 0.0;
  double reference_y = 0.0;
  ComputeReferenceDirection(weights, start_x, start_y, goal_x, goal_y, &reference_x, &reference_y);
  const double vref_n = std::hypot(reference_x, reference_y);

  const int n = grid.GetWidth() * grid.GetHeight();
  const int start = start_column + start_row * grid.GetWidth();
  const int goal = goal_column + goal_row * grid.GetWidth();
  std::vector<double> g(static_cast<size_t>(n), std::numeric_limits<double>::infinity());
  std::vector<int> parent(static_cast<size_t>(n), -1);
  std::vector<uint8_t> closed(static_cast<size_t>(n), 0);
  auto cmp = [](const Node& a, const Node& b) { return a.f > b.f; };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);
  g[static_cast<size_t>(start)] = 0.0;
  open.push(Node{weights.heuristic_weight * std::hypot(goal_x - start_x, goal_y - start_y), start});

  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const auto t0 = std::chrono::steady_clock::now();
  int expanded = 0;
  bool found = false;
  while (!open.empty() && expanded < weights.max_expansions) {
    if (weights.timeout_milliseconds > 0 && (expanded & 63) == 0) {
      const double ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t0)
                            .count();
      if (ms > weights.timeout_milliseconds) {
        break;
      }
    }
    const Node cur = open.top();
    open.pop();
    if (closed[static_cast<size_t>(cur.index)]) {
      continue;
    }
    closed[static_cast<size_t>(cur.index)] = 1;
    ++expanded;
    if (cur.index == goal) {
      found = true;
      break;
    }
    const int column_index = cur.index % grid.GetWidth();
    const int row_index = cur.index / grid.GetWidth();
    for (int k = 0; k < 8; ++k) {
      const int nx = column_index + dxs[k];
      const int ny = row_index + dys[k];
      if (nx < 0 || ny < 0 || nx >= grid.GetWidth() || ny >= grid.GetHeight()) {
        continue;
      }
      if (IsCellBlocked(grid, nx, ny, weights)) {
        continue;
      }
      const int ni = nx + ny * grid.GetWidth();
      if (closed[static_cast<size_t>(ni)]) {
        continue;
      }
      const double step = std::hypot(static_cast<double>(dxs[k]), static_cast<double>(dys[k])) *
                          grid.GetResolution();
      double cost = step;
      cost += weights.heat_weight * static_cast<double>(grid.GetCellHeat(nx, ny));
      const Cell cell = grid.GetCell(nx, ny);
      if (cell == Cell::kUnknown) {
        cost += weights.unknown_cell_cost * step;
      } else if (cell == Cell::kOccupied) {
        cost += weights.soft_occupied_cost;
      }
      if (vref_n > 1e-6 && (weights.alignment_weight > 0.0 || weights.crossing_weight > 0.0)) {
        const double sn = std::hypot(static_cast<double>(dxs[k]), static_cast<double>(dys[k]));
        const double dist_cells = std::hypot(nx - start_column, ny - start_row);
        const double decay =
            std::exp(-dist_cells / std::max(weights.alignment_decay_cell_count, 1e-3));
        const double align =
            1.0 - (dxs[k] * reference_x + dys[k] * reference_y) / (sn * vref_n);
        cost += weights.alignment_weight * std::max(0.0, align) * decay * grid.GetResolution();
        const double side = reference_x * dys[k] - reference_y * dxs[k];
        if (side < 0.0) {
          cost += weights.crossing_weight * decay * grid.GetResolution();
        }
      }
      const double ng = g[static_cast<size_t>(cur.index)] + cost;
      if (ng >= g[static_cast<size_t>(ni)]) {
        continue;
      }
      g[static_cast<size_t>(ni)] = ng;
      parent[static_cast<size_t>(ni)] = cur.index;
      const double h = std::hypot(goal_column - nx, goal_row - ny) * grid.GetResolution();
      open.push(Node{ng + weights.heuristic_weight * h, ni});
    }
  }
  if (!found) {
    return false;
  }
  return ReconstructSearchPath(grid, parent, start, goal, start_x, start_y, goal_x, goal_y, path);
}

bool PathSearcher::Jump(const OccupancyGrid& grid, double start_x, double start_y, double goal_x, double goal_y,
                      const SearchWeights& weights, std::vector<Eigen::Vector2d>* path) const {
  path->clear();
  int start_column = 0;
  int start_row = 0;
  int goal_column = 0;
  int goal_row = 0;
  if (!grid.ConvertWorldToCell(start_x, start_y, &start_column, &start_row) || !grid.ConvertWorldToCell(goal_x, goal_y, &goal_column, &goal_row)) {
    return false;
  }
  if (!IsJumpCellPassable(grid, start_column, start_row, weights) || !IsJumpCellPassable(grid, goal_column, goal_row, weights)) {
    return false;
  }
  const int n = grid.GetWidth() * grid.GetHeight();
  const int start = start_column + start_row * grid.GetWidth();
  const int goal = goal_column + goal_row * grid.GetWidth();
  std::vector<double> g(static_cast<size_t>(n), std::numeric_limits<double>::infinity());
  std::vector<int> parent(static_cast<size_t>(n), -1);
  std::vector<uint8_t> closed(static_cast<size_t>(n), 0);
  auto cmp = [](const Node& a, const Node& b) { return a.f > b.f; };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);
  g[static_cast<size_t>(start)] = 0.0;
  open.push(Node{std::hypot(goal_x - start_x, goal_y - start_y), start});
  const int dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  const auto t0 = std::chrono::steady_clock::now();
  int expanded = 0;
  bool found = false;
  while (!open.empty() && expanded < weights.max_expansions) {
    if (weights.timeout_milliseconds > 0 && (expanded & 31) == 0) {
      const double ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t0)
                            .count();
      if (ms > weights.timeout_milliseconds) {
        break;
      }
    }
    const Node cur = open.top();
    open.pop();
    if (closed[static_cast<size_t>(cur.index)]) {
      continue;
    }
    closed[static_cast<size_t>(cur.index)] = 1;
    ++expanded;
    if (cur.index == goal) {
      found = true;
      break;
    }
    const int column_index = cur.index % grid.GetWidth();
    const int row_index = cur.index / grid.GetWidth();
    for (const auto& dir : dirs) {
      int jx = 0;
      int jy = 0;
      if (!FindJumpPoint(grid, weights, column_index, row_index, dir[0], dir[1], goal_column, goal_row, &jx, &jy)) {
        continue;
      }
      const int ni = jx + jy * grid.GetWidth();
      if (closed[static_cast<size_t>(ni)]) {
        continue;
      }
      const double step = std::hypot(jx - column_index, jy - row_index) * grid.GetResolution();
      const double ng = g[static_cast<size_t>(cur.index)] + step;
      if (ng >= g[static_cast<size_t>(ni)]) {
        continue;
      }
      g[static_cast<size_t>(ni)] = ng;
      parent[static_cast<size_t>(ni)] = cur.index;
      const double h = std::hypot(goal_column - jx, goal_row - jy) * grid.GetResolution();
      open.push(Node{ng + h, ni});
    }
  }
  if (!found) {
    return false;
  }
  return ReconstructSearchPath(grid, parent, start, goal, start_x, start_y, goal_x, goal_y, path);
}

void PathSearcher::Shortcut(const OccupancyGrid& grid, std::vector<Eigen::Vector2d>* path, int line_of_sight_radius_cells,
                  double minimum_edge_length, double minimum_turn_degrees) const {
  if (path->size() < 3) {
    return;
  }
  std::vector<Eigen::Vector2d> los;
  los.push_back(path->front());
  size_t i = 0;
  while (i + 1 < path->size()) {
    size_t best = i + 1;
    for (size_t j = i + 2; j < path->size(); ++j) {
      if (!HasLineOfSight(grid, (*path)[i], (*path)[j], line_of_sight_radius_cells)) {
        break;
      }
      best = j;
    }
    los.push_back((*path)[best]);
    i = best;
  }
  std::vector<Eigen::Vector2d> spaced;
  spaced.push_back(los.front());
  for (size_t k = 1; k + 1 < los.size(); ++k) {
    const Eigen::Vector2d prev = spaced.back();
    const Eigen::Vector2d mid = los[k];
    const Eigen::Vector2d next = los[k + 1];
    const double len = (mid - prev).norm();
    const Eigen::Vector2d u = mid - prev;
    const Eigen::Vector2d v = next - mid;
    const double nu = u.norm();
    const double nv = v.norm();
    double turn = 180.0;
    if (nu > 1e-6 && nv > 1e-6) {
      const double c = std::max(-1.0, std::min(1.0, u.dot(v) / (nu * nv)));
      turn = std::acos(c) * 180.0 / 3.14159265358979323846;
    }
    if (!(len < minimum_edge_length && turn < minimum_turn_degrees)) {
      spaced.push_back(mid);
    }
  }
  spaced.push_back(los.back());
  *path = std::move(spaced);
}

namespace {

/**
 * @brief Drop interior points whose incoming and outgoing steps differ by less than 1e-2 in taxicab length.
 * @param path Polyline rewritten in place. Fewer than three points is a no-op.
 */
void RemoveCollinearPoints(std::vector<Eigen::Vector2d>* path) {
  if (path->size() < 3) {
    return;
  }
  std::vector<Eigen::Vector2d> kept;
  kept.push_back(path->front());
  for (size_t i = 1; i + 1 < path->size(); ++i) {
    const Eigen::Vector2d turn =
        ((*path)[i + 1] - (*path)[i]) - ((*path)[i] - (*path)[i - 1]);
    if (std::abs(turn.x()) + std::abs(turn.y()) > 1e-2) {
      kept.push_back((*path)[i]);
    }
  }
  kept.push_back(path->back());
  *path = std::move(kept);
}

/**
 * @brief Replace a corner by its chord when the chord heat does not exceed the corner heat by more than 0.50.
 *
 * heat_weight of 0 returns immediately. The scan is one direction; the caller
 * reverses the polyline and scans again.
 *
 * @param grid Heat field.
 * @param path Polyline rewritten in place.
 * @param heat_weight Zero disables the cut. Any positive value enables it.
 */
void RemoveHeatCorners(const OccupancyGrid& grid, std::vector<Eigen::Vector2d>* path,
                        double heat_weight) {
  if (path->size() < 3) {
    return;
  }
  const bool heat_on = heat_weight > 0.0;
  const double ds = std::max(0.5 * grid.GetResolution(), 0.05);
  constexpr double kPeakRelax = 0.50;
  auto segment = [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b, double* peak) {
    if (peak != nullptr) {
      *peak = 0.0;
    }
    if (!HasLineOfSight(grid, a, b, 0)) {
      return std::numeric_limits<double>::infinity();
    }
    const double length = (b - a).norm();
    if (length < 1e-9) {
      return 0.0;
    }
    double cost = length;
    if (!heat_on) {
      return cost;
    }
    const Eigen::Vector2d dir = (b - a) / length;
    double heat_int = 0.0;
    double peak_h = 0.0;
    for (double s = 0.0; s <= length; s += ds) {
      const Eigen::Vector2d p = a + s * dir;
      int column_index = 0;
      int row_index = 0;
      const float heat = grid.ConvertWorldToCell(p.x(), p.y(), &column_index, &row_index) ? grid.GetCellHeat(column_index, row_index) : 0.0f;
      peak_h = std::max(peak_h, static_cast<double>(heat));
      heat_int += static_cast<double>(heat) * ds;
    }
    if (peak != nullptr) {
      *peak = peak_h;
    }
    return cost + heat_weight * heat_int;
  };

  std::vector<Eigen::Vector2d> kept;
  kept.push_back(path->front());
  double peak1 = 0.0;
  double cost1 = segment((*path)[0], (*path)[1], &peak1);
  Eigen::Vector2d prev = path->front();
  for (size_t i = 1; i + 1 < path->size(); ++i) {
    const Eigen::Vector2d pose1 = (*path)[i];
    const Eigen::Vector2d pose2 = (*path)[i + 1];
    double peak2 = 0.0;
    double peak3 = 0.0;
    const double cost2 = segment(pose1, pose2, &peak2);
    const double cost3 = segment(prev, pose2, &peak3);
    bool accept = cost3 < cost1 + cost2;
    if (accept && heat_on && peak3 > std::max(peak1, peak2) * (1.0 + kPeakRelax)) {
      accept = false;
    }
    if (accept) {
      cost1 = cost3;
      peak1 = peak3;
    } else {
      kept.push_back(pose1);
      prev = pose1;
      cost1 = segment(pose1, pose2, &peak1);
    }
  }
  kept.push_back(path->back());
  *path = std::move(kept);
}

}  // namespace

void PathSearcher::Prune(const OccupancyGrid& grid, std::vector<Eigen::Vector2d>* path, double heat_weight) const {
  RemoveCollinearPoints(path);
  RemoveHeatCorners(grid, path, heat_weight);
  std::reverse(path->begin(), path->end());
  RemoveHeatCorners(grid, path, heat_weight);
  std::reverse(path->begin(), path->end());
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
