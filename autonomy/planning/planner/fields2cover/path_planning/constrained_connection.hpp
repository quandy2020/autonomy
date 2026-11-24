/* Copyright 2026 Autodriver contributors. Licensed under Apache-2.0. */
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include "autonomy/planning/planner/fields2cover/path_planning/reeds_shepp_curves.hpp"

namespace autonomy::planning::planner::fields2cover::pp::detail {

// Prepared ring coordinates keep collision checking independent of repeatedly
// allocating native geometry objects for every steering sample.

/**
 * @class FreeSpace
 * @brief Polygon free space used by the steering search and link checks.
 *
 * The constructor copies every ring into plain doubles, so the hot
 * Contains() path performs no native geometry calls and no allocation. Each
 * ring keeps a bounding box that rejects far-away candidates with two
 * comparisons, which keeps the point test cheap on maps with many polygons.
 */
class FreeSpace {
 public:
  /**
   * @brief Flattens the cells into cached ring coordinates.
   * @param[in] cells Free-space polygons; holes follow their outer ring.
   */
  explicit FreeSpace(const F2CCells& cells) {
    for (const auto& cell : cells) {
      std::vector<Ring> rings;
      for (const auto& ring : cell) {
        Ring cached;
        for (size_t i = 0; i < ring.size(); ++i) {
          cached.points.push_back({ring[i].GetX(), ring[i].GetY()});
        }
        if (cached.points.size() <= 2) {
          continue;
        }
        cached.box = BoxOf(cached.points);
        rings.push_back(std::move(cached));
      }
      if (!rings.empty()) {
        cells_.push_back(std::move(rings));
      }
    }
    min_x = cells.GetDimMinX();
    min_y = cells.GetDimMinY();
    max_x = cells.GetDimMaxX();
    max_y = cells.GetDimMaxY();
  }

  /**
   * @brief Point-in-free-space test.
   * @param[in] p Query point in map coordinates.
   * @return True when @p p is inside a cell and outside every hole.
   */
  bool Contains(const F2CPoint& p) const {
    return Contains(p.GetX(), p.GetY());
  }

  /**
   * @brief Point-in-free-space test on raw coordinates.
   * @param[in] x Query abscissa in map coordinates.
   * @param[in] y Query ordinate in map coordinates.
   * @return True when the point is inside a cell and outside every hole.
   */
  bool Contains(double x, double y) const {
    if (x < min_x - 1e-8 || x > max_x + 1e-8 || y < min_y - 1e-8 ||
        y > max_y + 1e-8) {
      return false;
    }
    for (const auto& rings : cells_) {
      if (!InsideRing(rings.front(), x, y)) {
        continue;
      }
      bool hole = false;
      for (size_t i = 1; i < rings.size(); ++i) {
        if (InsideRing(rings[i], x, y)) {
          hole = true;
          break;
        }
      }
      if (!hole) {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Straight-line test used by the steering validator.
   * @param[in] a Segment start point.
   * @param[in] b Segment end point.
   * @return True when the open segment stays inside the free space.
   */
  bool Segment(const F2CPoint& a, const F2CPoint& b) const {
    if (!Contains(a) || !Contains(b) || !Contains((a + b) * .5)) {
      return false;
    }
    const double ax = a.GetX();
    const double ay = a.GetY();
    const double bx = b.GetX();
    const double by = b.GetY();
    // End points can lie inside while a long straight cuts a concave boundary
    // or a small hole. Check every boundary edge as well as the midpoint.
    for (const auto& rings : cells_)
      for (const auto& ring : rings) {
        if (ring.points.size() < 2) {
          continue;
        }
        for (size_t i = 0, j = ring.points.size() - 1; i < ring.points.size();
             j = i++) {
          const double xj = ring.points[j][0];
          const double yj = ring.points[j][1];
          const double xi = ring.points[i][0];
          const double yi = ring.points[i][1];
          const double a1 = Cross(ax, ay, bx, by, xj, yj);
          const double a2 = Cross(ax, ay, bx, by, xi, yi);
          const double b1 = Cross(xj, yj, xi, yi, ax, ay);
          const double b2 = Cross(xj, yj, xi, yi, bx, by);
          if (((a1 > 1e-9 && a2 < -1e-9) || (a1 < -1e-9 && a2 > 1e-9)) &&
              ((b1 > 1e-9 && b2 < -1e-9) || (b1 < -1e-9 && b2 > 1e-9))) {
            return false;
          }
        }
      }
    return true;
  }

  double min_x, min_y, max_x, max_y;

 private:
  struct Box {
    double min_x, min_y, max_x, max_y;
  };
  struct Ring {
    std::vector<std::array<double, 2>> points;
    Box box{0.0, 0.0, 0.0, 0.0};
  };

  static Box BoxOf(const std::vector<std::array<double, 2>>& points) {
    Box box{points.front()[0], points.front()[1], points.front()[0],
            points.front()[1]};
    for (const auto& point : points) {
      box.min_x = std::min(box.min_x, point[0]);
      box.min_y = std::min(box.min_y, point[1]);
      box.max_x = std::max(box.max_x, point[0]);
      box.max_y = std::max(box.max_y, point[1]);
    }
    return box;
  }

  static double Cross(double ax, double ay, double bx, double by, double px,
                      double py) {
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
  }

  /**
   * @brief Even-odd point in ring test with a bounding box prefilter.
   * @param[in] ring Cached ring coordinates and its bounding box.
   * @param[in] x Query abscissa.
   * @param[in] y Query ordinate.
   * @return True when the point lies inside or on the ring boundary.
   */
  static bool InsideRing(const Ring& ring, double x, double y) {
    const Box& box = ring.box;
    if (x < box.min_x - 1e-8 || x > box.max_x + 1e-8 || y < box.min_y - 1e-8 ||
        y > box.max_y + 1e-8) {
      return false;
    }
    bool inside = false;
    const auto& points = ring.points;
    for (size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
      const double ax = points[j][0];
      const double ay = points[j][1];
      const double bx = points[i][0];
      const double by = points[i][1];
      if (std::fabs(Cross(ax, ay, bx, by, x, y)) < 1e-8 &&
          x >= std::min(ax, bx) - 1e-8 && x <= std::max(ax, bx) + 1e-8 &&
          y >= std::min(ay, by) - 1e-8 && y <= std::max(ay, by) + 1e-8) {
        return true;
      }
      if ((ay > y) != (by > y) &&
          x < (bx - ax) * (y - ay) / (by - ay) + ax) {
        inside = !inside;
      }
    }
    return inside;
  }

  std::vector<std::vector<Ring>> cells_;
};

inline bool ValidPath(const F2CPath& path, const F2CRobot& robot,
    const FreeSpace* free = nullptr) {
  if (path.size() == 0) return false;
  for (size_t i = 0; i < path.size(); ++i) {
    const auto& s = path[i];
    if (!std::isfinite(s.point.GetX()) || !std::isfinite(s.point.GetY()) ||
        !std::isfinite(s.angle) || !std::isfinite(s.len) || s.len < 0.0) return false;
    if (free && !free->Segment(s.point, s.AtEnd())) return false;
    if (i + 1 < path.size()) {
      if (s.AtEnd().Distance(path[i + 1].point) > .01) return false;
      if (s.dir == path[i + 1].dir && s.len > 1e-5 &&
          F2CPoint::GetAngleDiffAbs(s.angle, path[i + 1].angle) /
              s.len > 1.01 / robot.GetMinTurningRadius()) return false;
    }
  }
  return true;
}

struct Pose { F2CPoint p; double angle; };
inline Pose Advance(const Pose& pose, double length, double curvature, int dir) {
  const double ds = dir * length;
  Pose next{pose.p, F2CPoint::Mod2pi(pose.angle + ds * curvature)};
  if (std::fabs(curvature) < 1e-10) {
    next.p = pose.p + F2CPoint(ds * std::cos(pose.angle), ds * std::sin(pose.angle));
  } else {
    next.p = pose.p + F2CPoint((std::sin(pose.angle + ds * curvature) - std::sin(pose.angle)) / curvature,
        (std::cos(pose.angle) - std::cos(pose.angle + ds * curvature)) / curvature);
  }
  return next;
}

inline F2CPath Primitive(const Pose& start, double length, double curvature,
    int dir, const F2CRobot& robot) {
  F2CPath path;
  const int count = std::max(1, static_cast<int>(std::ceil(length / .05)));
  const double step = length / count;
  Pose pose = start;
  for (int i = 0; i < count; ++i) {
    Pose next = Advance(pose, step, curvature, dir);
    path.AddState(pose.p, pose.angle, pose.p.Distance(next.p),
        static_cast<types::PathDirection>(dir), types::PathSectionType::TURN, robot.GetTurnVel());
    pose = next;
  }
  path.AddState(pose.p, pose.angle, 0.0, static_cast<types::PathDirection>(dir),
      types::PathSectionType::TURN, robot.GetTurnVel());
  return path;
}

// A 2-D distance map guides the pose search around concave boundaries and holes.
// Each search edge is an integrated steering primitive, and the final edge is
// produced by the selected turn planner with the exact goal pose.
inline F2CPath SearchConnection(const F2CRobot& robot, TurningBase& turn,
    const F2CPoint& start, double start_angle, const F2CPoint& goal,
    double goal_angle, const FreeSpace& free) {
  auto direct = turn.CreateTurn(robot, start, start_angle, goal, goal_angle);
  if (ValidPath(direct, robot, &free)) return direct;
  if (turn.HasContinuousCurvature())
    throw std::runtime_error("No collision-free continuous-curvature connection");
  const double radius = robot.GetMinTurningRadius();
  const double grid = std::max(.75 * radius,
      std::hypot(free.max_x - free.min_x, free.max_y - free.min_y) / 400.0);
  const int nx = static_cast<int>(std::ceil((free.max_x - free.min_x) / grid)) + 1;
  const int ny = static_cast<int>(std::ceil((free.max_y - free.min_y) / grid)) + 1;
  auto PointAt = [&](int id) {return F2CPoint(free.min_x + (id % nx) * grid,
      free.min_y + (id / nx) * grid);};
  auto GridId = [&](const F2CPoint& p) {
    return std::clamp(static_cast<int>(std::lround((p.GetY() - free.min_y) / grid)), 0, ny - 1) * nx +
        std::clamp(static_cast<int>(std::lround((p.GetX() - free.min_x) / grid)), 0, nx - 1);
  };
  std::vector<bool> open(nx * ny);
  for (int i = 0; i < nx * ny; ++i) open[i] = free.Contains(PointAt(i));
  std::vector<double> distance(nx * ny, std::numeric_limits<double>::infinity());
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
  const int end = GridId(goal); distance[end] = 0.0; open[end] = true; queue.emplace(0.0, end);
  while (!queue.empty()) {
    auto [cost, id] = queue.top(); queue.pop();
    if (cost > distance[id] + 1e-9) continue;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
      const int x = id % nx + dx, y = id / nx + dy;
      if ((dx == 0 && dy == 0) || x < 0 || y < 0 || x >= nx || y >= ny) continue;
      const int next = y * nx + x;
      const double value = cost + grid * std::hypot(dx, dy);
      if (open[next] && value < distance[next] && free.Segment(PointAt(id), PointAt(next))) {
        distance[next] = value; queue.emplace(value, next);
      }
    }
  }
  auto Heuristic = [&](const F2CPoint& p) {
    const double d = distance[GridId(p)];
    return std::isfinite(d) ? std::max(p.Distance(goal), d - grid) : p.Distance(goal);
  };
  struct Node { Pose pose; double cost; int parent; double curvature; int dir; };
  std::vector<Node> nodes{{{start, start_angle}, 0.0, -1, 0.0, 1}};
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> frontier;
  frontier.emplace(Heuristic(start), 0);
  const double key_grid = .35 * radius;
  auto Key = [&](const Pose& pose, int dir) {
    const int x = static_cast<int>(std::lround((pose.p.GetX() - free.min_x) / key_grid));
    const int y = static_cast<int>(std::lround((pose.p.GetY() - free.min_y) / key_grid));
    const int a = static_cast<int>(std::lround(F2CPoint::Mod2pi(pose.angle) * 72.0 / (2.0 * M_PI))) % 72;
    return (static_cast<uint64_t>(y) << 32) | (static_cast<uint64_t>(x) << 8) |
        static_cast<uint64_t>(a * 2 + (dir < 0));
  };
  std::unordered_map<uint64_t, double> best;
  best.emplace(Key(nodes[0].pose, 1), 0.0);
  const bool reverse = dynamic_cast<ReedsSheppCurves*>(&turn) != nullptr;
  const double length = .8 * radius;
  const auto began = std::chrono::steady_clock::now();
  size_t expanded = 0;
  while (!frontier.empty() && expanded < 100000) {
    auto [priority, id] = frontier.top(); frontier.pop();
    const Node current = nodes[id];
    if (current.cost > best[Key(current.pose, current.dir)] + 1e-9) continue;
    ++expanded;
    if (expanded % 256 == 0 && std::chrono::steady_clock::now() - began > std::chrono::seconds(15)) break;
    if (expanded % 8 == 0 || current.pose.p.Distance(goal) < 3.0 * radius) {
      auto tail = turn.CreateTurn(robot, current.pose.p, current.pose.angle, goal, goal_angle);
      if (ValidPath(tail, robot, &free)) {
        std::vector<int> chain;
        for (int n = id; nodes[n].parent >= 0; n = nodes[n].parent) chain.push_back(n);
        F2CPath result;
        for (auto n = chain.rbegin(); n != chain.rend(); ++n) {
          const Node& edge = nodes[*n];
          result += Primitive(nodes[edge.parent].pose, length, edge.curvature, edge.dir, robot);
        }
        result += tail;
        if (ValidPath(result, robot, &free)) return result;
      }
    }
    for (int dir : {1, -1}) {
      if (dir < 0 && !reverse) continue;
      for (double steering : {-1.0, -.5, 0.0, .5, 1.0}) {
        const double curvature = steering / (1.01 * radius);
        Pose next = Advance(current.pose, length, curvature, dir);
        const double cost = current.cost + length * (dir < 0 ? 1.2 : 1.0) +
            (dir != current.dir ? .5 * radius : 0.0);
        const uint64_t key = Key(next, dir);
        auto it = best.find(key);
        if (it != best.end() && cost >= it->second - 1e-9) continue;
        auto edge = Primitive(current.pose, length, curvature, dir, robot);
        if (!ValidPath(edge, robot, &free)) continue;
        best[key] = cost;
        nodes.push_back({next, cost, id, curvature, dir});
        frontier.emplace(cost + 1.5 * Heuristic(next.p), static_cast<int>(nodes.size() - 1));
      }
    }
  }
  throw std::runtime_error("No collision-free bounded-curvature connection within search budget");
}

}  // namespace autonomy::planning::planner::fields2cover::pp::detail
