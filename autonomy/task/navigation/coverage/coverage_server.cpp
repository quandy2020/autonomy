// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_server.cpp
 * @brief Native complete-coverage coverage server implementation. */
#include "autonomy/task/navigation/coverage/coverage_server.hpp"

#include "autonomy/task/navigation/coverage/map_connector.hpp"
#include "autonomy/task/navigation/coverage/region_orderer.hpp"
#include "autonomy/task/navigation/coverage/region_segmenter.hpp"

#include <boost/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "autonomy/planning/planner/fields2cover/path_planning/constrained_connection.hpp"

namespace autonomy::task::coverage {
namespace {

using autonomy::planning::planner::fields2cover::pp::detail::FreeSpace;


void Validate(const F2CCells& cells) {
  using BPoint = boost::geometry::model::d2::point_xy<double>;
  using BPolygon = boost::geometry::model::polygon<BPoint>;
  if (cells.IsEmpty() || cells.Area() < 1e-6)
    throw RequestException("Empty field");
  size_t count = 0;
  for (const auto& cell : cells) {
    for (const auto& ring : cell)
      for (const auto& p : ring) {
        if (!std::isfinite(p.GetX()) || !std::isfinite(p.GetY()))
          throw RequestException("Nonfinite coordinate");
        if (++count > 4096)
          throw RequestException("Field has more than 4096 vertices");
      }
    BPolygon polygon;
    boost::geometry::read_wkt(cell.ExportToWkt(), polygon);
    boost::geometry::correct(polygon);
    std::string reason;
    if (!boost::geometry::is_valid(polygon, reason))
      throw RequestException("Invalid field topology: " + reason);
  }
}
F2CPoint OutputPoint(const F2CPoint& point, const F2CField& field,
                     bool cartesian) {
  auto result = point + field.GetRefPoint();
  if (!cartesian)
    result =
        f2c::Transform::transform(result, field.GetCRS(), field.GetPrevCRS());
  return result;
}
void Pose(NavPath& path, const F2CPoint& point, double angle,
          const std::string& frame) {
  auto* pose = path.add_poses();
  pose->mutable_header()->set_frame_id(frame);
  auto* p = pose->mutable_pose()->mutable_position();
  p->set_x(point.GetX());
  p->set_y(point.GetY());
  p->set_z(point.GetZ());
  auto* q = pose->mutable_pose()->mutable_orientation();
  q->set_z(std::sin(angle / 2));
  q->set_w(std::cos(angle / 2));
}
using CleanPoint = std::pair<double, double>;

bool NearlyEqual(const CleanPoint& a, const CleanPoint& b) {
  return std::hypot(a.first - b.first, a.second - b.second) < 1e-8;
}

bool IsSpike(const CleanPoint& previous, const CleanPoint& current,
             const CleanPoint& next) {
  const double ax = previous.first - current.first;
  const double ay = previous.second - current.second;
  const double bx = next.first - current.first;
  const double by = next.second - current.second;
  const double cross = ax * by - ay * bx;
  const double dot = ax * bx + ay * by;
  return std::abs(cross) < 1e-9 && dot < 0.0;
}

std::vector<CleanPoint> CleanRing(const F2CLinearRing& ring) {
  std::vector<CleanPoint> points;
  for (const auto& point : ring) {
    const CleanPoint value{point.GetX(), point.GetY()};
    if (points.empty() || !NearlyEqual(points.back(), value)) {
      points.push_back(value);
    }
  }
  if (points.size() > 1 && NearlyEqual(points.front(), points.back())) {
    points.pop_back();
  }
  bool changed = true;
  while (changed && points.size() > 3) {
    changed = false;
    for (std::size_t index = 0; index < points.size(); ++index) {
      const auto& previous =
          points[(index + points.size() - 1) % points.size()];
      const auto& current = points[index];
      const auto& next = points[(index + 1) % points.size()];
      if (IsSpike(previous, current, next)) {
        points.erase(points.begin() + static_cast<std::ptrdiff_t>(index));
        changed = true;
        break;
      }
    }
  }
  double signed_area = 0.0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto& current = points[index];
    const auto& next = points[(index + 1) % points.size()];
    signed_area += current.first * next.second - next.first * current.second;
  }
  if (signed_area < 0.0) {
    std::reverse(points.begin(), points.end());
  }
  return points;
}

F2CSwaths Flatten(const F2CRoute& route) {
  F2CSwaths result;
  for (const auto& group : route.GetVectorSwaths()) result.Append(group);
  return result;
}

/** @brief Append one straight state so the exporter samples the whole line. */
void AddLine(F2CPath& path, const F2CPoint& from, const F2CPoint& to,
             f2c::types::PathSectionType type, double velocity) {
  const double length = from.Distance(to);
  if (length < 1e-9) {
    return;
  }
  const double heading = std::atan2(to.GetY() - from.GetY(),
                                    to.GetX() - from.GetX());
  path.AddState(from, heading, length, f2c::types::PathDirection::FORWARD, type,
                velocity);
}

/** @brief Check a segment by sampling the free-space predicate. */
bool SegmentClear(const FreeSpace& free, const F2CPoint& from,
                  const F2CPoint& to) {
  const double length = from.Distance(to);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / 0.02)));
  for (int step = 0; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / steps;
    const F2CPoint point(from.GetX() + (to.GetX() - from.GetX()) * ratio,
                         from.GetY() + (to.GetY() - from.GetY()) * ratio);
    if (!free.Contains(point)) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Find a collision-free polyline between two points in a room.
 *
 * A strip change can happen across a concave room boundary.  The old code
 * joined those strip ends with one straight line, which is fine for a convex
 * room but cuts through the notch of an L- or U-shaped room.  Search a small
 * eight-connected grid and then remove redundant vertices with a visibility
 * check.  The grid is only used for the rare long link; normal U-turns still
 * use the analytic half-circle below.
 */
std::vector<F2CPoint> RouteAround(const F2CPoint& from, const F2CPoint& to,
                                  const FreeSpace& free) {
  const double span_x = free.max_x - free.min_x;
  const double span_y = free.max_y - free.min_y;
  if (!(span_x > 1e-6) || !(span_y > 1e-6)) {
    return {};
  }
  constexpr double kResolution = 0.05;
  const int nx = static_cast<int>(std::ceil(span_x / kResolution)) + 1;
  const int ny = static_cast<int>(std::ceil(span_y / kResolution)) + 1;
  const int count = nx * ny;
  if (count <= 0) {
    return {};
  }
  const auto point_at = [&](int id) {
    return F2CPoint(free.min_x + (id % nx) * kResolution,
                    free.min_y + (id / nx) * kResolution);
  };
  std::vector<unsigned char> open(static_cast<std::size_t>(count));
  for (int id = 0; id < count; ++id) {
    open[static_cast<std::size_t>(id)] = free.Contains(point_at(id));
  }
  const auto nearest_open = [&](const F2CPoint& point) {
    int best = -1;
    double best_distance = std::numeric_limits<double>::max();
    for (int id = 0; id < count; ++id) {
      if (!open[static_cast<std::size_t>(id)]) {
        continue;
      }
      const double distance = point.Distance(point_at(id));
      if (distance < best_distance) {
        best_distance = distance;
        best = id;
      }
    }
    return best;
  };
  const int start = nearest_open(from);
  const int goal = nearest_open(to);
  if (start < 0 || goal < 0) {
    std::fprintf(stderr, "route: no open grid start=%d goal=%d\n", start, goal);
    return {};
  }
  std::vector<double> cost(static_cast<std::size_t>(count),
                           std::numeric_limits<double>::infinity());
  std::vector<int> parent(static_cast<std::size_t>(count), -1);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> frontier;
  cost[static_cast<std::size_t>(start)] = 0.0;
  frontier.emplace(0.0, start);
  constexpr int kDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  constexpr int kDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  while (!frontier.empty()) {
    const auto [priority, id] = frontier.top();
    frontier.pop();
    if (priority > cost[static_cast<std::size_t>(id)] + 1e-9) {
      continue;
    }
    if (id == goal) {
      break;
    }
    const int x = id % nx;
    const int y = id / nx;
    for (int direction = 0; direction < 8; ++direction) {
      const int next_x = x + kDx[direction];
      const int next_y = y + kDy[direction];
      if (next_x < 0 || next_y < 0 || next_x >= nx || next_y >= ny) {
        continue;
      }
      const int next = next_y * nx + next_x;
      if (!open[static_cast<std::size_t>(next)]) {
        continue;
      }
      // Do not squeeze diagonally through a one-cell corner.
      if (kDx[direction] != 0 && kDy[direction] != 0 &&
          (!open[static_cast<std::size_t>(y * nx + next_x)] ||
           !open[static_cast<std::size_t>(next_y * nx + x)])) {
        continue;
      }
      if (!SegmentClear(free, point_at(id), point_at(next))) {
        continue;
      }
      const double next_cost =
          cost[static_cast<std::size_t>(id)] + kResolution *
              std::hypot(static_cast<double>(kDx[direction]),
                         static_cast<double>(kDy[direction]));
      if (next_cost + 1e-9 >= cost[static_cast<std::size_t>(next)]) {
        continue;
      }
      cost[static_cast<std::size_t>(next)] = next_cost;
      parent[static_cast<std::size_t>(next)] = id;
      frontier.emplace(next_cost, next);
    }
  }
  if (start != goal && parent[static_cast<std::size_t>(goal)] < 0) {
    std::fprintf(stderr, "route: no path\n");
    return {};
  }
  std::vector<F2CPoint> raw;
  for (int id = goal;; id = parent[static_cast<std::size_t>(id)]) {
    raw.push_back(point_at(id));
    if (id == start) {
      break;
    }
  }
  // Do not std::reverse a vector<F2CPoint>: Point's move assignment reads the
  // source, and a moved-from Point has already released its native geometry.
  // Copy the nodes in reverse order instead.
  std::vector<F2CPoint> forward;
  forward.reserve(raw.size());
  for (auto it = raw.rbegin(); it != raw.rend(); ++it) {
    forward.push_back(*it);
  }
  if (!SegmentClear(free, from, forward.front()) ||
      !SegmentClear(free, forward.back(), to)) {
    std::fprintf(stderr, "route: endpoint segment failed from=%d to=%d\n",
                 SegmentClear(free, from, forward.front()),
                 SegmentClear(free, forward.back(), to));
    return {};
  }
  std::vector<F2CPoint> points;
  points.push_back(from);
  points.insert(points.end(), forward.begin(), forward.end());
  points.push_back(to);
  if (points.size() < 2) {
    return {};
  }
  std::vector<F2CPoint> simplified;
  std::size_t current = 0;
  while (current + 1 < points.size()) {
    std::size_t next = points.size() - 1;
    for (; next > current + 1; --next) {
      if (SegmentClear(free, points[current], points[next])) {
        break;
      }
    }
    simplified.push_back(points[current]);
    current = next;
  }
  simplified.push_back(points.back());
  return simplified;
}

/** @brief Append a straight link, or a wall-following detour when needed. */
bool AddTraversableLink(F2CPath& path, const F2CPoint& from,
                        const F2CPoint& to, const FreeSpace& free,
                        double velocity) {
  if (from.Distance(to) < 1e-9) {
    return true;
  }
  const bool direct = SegmentClear(free, from, to);
  if (direct) {
    AddLine(path, from, to, f2c::types::PathSectionType::TURN, velocity);
    return true;
  }
  const auto route = RouteAround(from, to, free);
  if (route.size() < 2) {
    AddLine(path, from, to, f2c::types::PathSectionType::TURN, velocity);
    return false;
  }
  for (std::size_t index = 1; index < route.size(); ++index) {
    AddLine(path, route[index - 1], route[index],
            f2c::types::PathSectionType::TURN, velocity);
  }
  std::fprintf(stderr,
               "coverage: routed link %.2f,%.2f -> %.2f,%.2f via %zu points\n",
               from.GetX(), from.GetY(), to.GetX(), to.GetY(), route.size());
  return true;
}

/**
 * @brief Add a half-circle U-turn between two neighbouring swath ends.
 *
 * The arc uses half the gap between the two lines, which is exactly the shape
 * of the reference boustrophedon sweep: the pass ends, the robot curves
 * through the gap and starts the next pass facing the other way. The arc is
 * sampled into short segments so the exported path is smooth.
 * @return True when every sampled point stays inside @p free.
 */
bool AddUTurn(F2CPath& path, const F2CPoint& from, const F2CPoint& to,
              const F2CCells& free, double velocity) {
  const double chord = from.Distance(to);
  if (chord < 1e-6) {
    return false;
  }
  // A pass has two ways round the same chord. Try the one that stays inside
  // the room; trying only one direction is why half of the joints used to fall
  // back to a pivot turn.
  for (const double sweep : {M_PI, -M_PI}) {
    const double radius = 0.5 * chord;
    const F2CPoint center((from.GetX() + to.GetX()) * 0.5,
                          (from.GetY() + to.GetY()) * 0.5);
    const double start_angle =
        std::atan2(from.GetY() - center.GetY(), from.GetX() - center.GetX());
    const int steps = std::min(
        160, std::max(12, static_cast<int>(std::ceil(std::abs(sweep) * radius /
                                                     0.01))));
    F2CPath candidate;
    F2CPoint previous = from;
    bool inside = true;
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      const double angle = start_angle + sweep * ratio;
      const F2CPoint current(center.GetX() + radius * std::cos(angle),
                             center.GetY() + radius * std::sin(angle));
      if (!free.IsPointIn(current)) {
        inside = false;
        break;
      }
      AddLine(candidate, previous, current, f2c::types::PathSectionType::TURN,
              velocity);
      previous = current;
    }
    if (inside) {
      path += candidate;
      return true;
    }
  }
  return false;
}

/**
 * @brief Order swaths so that consecutive passes share an end point.
 *
 * The cell decomposition splits a room into strips and the upstream route
 * visits those strips row by row, which leaves four metre hops between passes
 * that are far too long for a half-circle U-turn and cut straight through
 * walls. Walking the passes from the nearest end and then removing the
 * remaining long hops with 2-opt keeps every join one row spacing wide, so the
 * turn planner can draw an arc.
 */
F2CSwaths OrderSweep(const F2CSwaths& swaths) {
  if (swaths.size() < 3) {
    return swaths;
  }
  const F2CPoint entry = swaths[0].StartPoint();
  std::vector<F2CSwath> route;
  route.reserve(swaths.size());
  std::vector<F2CSwath> remaining;
  remaining.reserve(swaths.size());
  for (const auto& swath : swaths) {
    remaining.push_back(swath);
  }
  F2CPoint current = entry;
  bool have_current = false;
  while (!remaining.empty()) {
    std::size_t best = 0;
    bool flip = false;
    double best_cost = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < remaining.size(); ++index) {
      const double to_start =
          have_current ? current.Distance(remaining[index].StartPoint()) : 0.0;
      const double to_end =
          have_current ? current.Distance(remaining[index].EndPoint()) : 0.0;
      if (to_start < best_cost) {
        best_cost = to_start;
        best = index;
        flip = false;
      }
      if (to_end < best_cost) {
        best_cost = to_end;
        best = index;
        flip = true;
      }
    }
    F2CSwath chosen = remaining[best];
    if (flip) {
      chosen.Reverse();
    }
    current = chosen.EndPoint();
    have_current = true;
    route.push_back(chosen);
    remaining.erase(remaining.begin() +
                    static_cast<std::vector<F2CSwath>::difference_type>(best));
  }
  // The greedy walk can still finish in a corner and leave a long hop back to
  // an unvisited strip. Reversing a run of passes removes those hops.
  const auto distance = [](const F2CPoint& a, const F2CPoint& b) {
    return a.Distance(b);
  };
  for (int pass = 0; pass < 6; ++pass) {
    bool improved = false;
    for (std::size_t head = 0; head + 1 < route.size(); ++head) {
      for (std::size_t tail = head + 1; tail < route.size(); ++tail) {
        const F2CPoint before =
            head == 0 ? entry : route[head - 1].EndPoint();
        const F2CPoint after =
            tail + 1 < route.size() ? route[tail + 1].StartPoint() : F2CPoint();
        const bool has_after = tail + 1 < route.size();
        const double old_cost =
            distance(before, route[head].StartPoint()) +
            (has_after ? distance(route[tail].EndPoint(), after) : 0.0);
        const double new_cost =
            distance(before, route[tail].EndPoint()) +
            (has_after ? distance(route[head].StartPoint(), after) : 0.0);
        if (new_cost + 1e-6 < old_cost) {
          std::reverse(route.begin() + static_cast<std::ptrdiff_t>(head),
                       route.begin() + static_cast<std::ptrdiff_t>(tail) + 1);
          for (std::size_t index = head; index <= tail; ++index) {
            route[index].Reverse();
          }
          improved = true;
        }
      }
    }
    if (!improved) {
      break;
    }
  }
  F2CSwaths ordered;
  for (const auto& swath : route) {
    ordered.push_back(swath);
  }
  return ordered;
}

/**
 * @brief Build a boustrophedon path: straight passes joined by U-turns.
 *
 * Fields2Cover's connection search gives up on dense spacing because a
 * bounded-curvature turn cannot fit between two passes that are closer than
 * the turning diameter. Splicing the passes ourselves reproduces the reference
 * sweep and keeps the planning time linear.
 */
F2CPath BuildSweepPath(const F2CSwaths& swaths, const F2CCells& free,
                       double velocity, int* smooth_turns, int* tight_turns,
                       int* link_turns) {
  F2CPath path;
  const FreeSpace free_space(free);
  F2CPoint previous_end;
  bool have_previous = false;
  for (const auto& swath : swaths) {
    const auto& line = swath.GetPath();
    if (line.size() < 2 || line.Length() < 1e-6) {
      continue;
    }
    F2CPoint start = line.StartPoint();
    F2CPoint end = line.EndPoint();
    // Leave a small gap at both ends: the half-circle U-turn needs that space
    // and the reference sweep shows the same rounded corner at each pass end.
    const double length = start.Distance(end);
    const double trim = std::min(0.12, 0.45 * length);
    if (length > 1e-6 && trim > 1e-6) {
      const F2CPoint direction((end.GetX() - start.GetX()) / length,
                               (end.GetY() - start.GetY()) / length);
      start = F2CPoint(start.GetX() + direction.GetX() * trim,
                       start.GetY() + direction.GetY() * trim);
      end = F2CPoint(end.GetX() - direction.GetX() * trim,
                     end.GetY() - direction.GetY() * trim);
    }
    if (have_previous) {
      F2CPath turn;
      // Only wrap a short gap: a long chord would draw a huge circle.
      bool turned = previous_end.Distance(start) <= 0.9 &&
                    AddUTurn(turn, previous_end, start, free, velocity);
      if (!turned) {
        // Consecutive passes of different strips do not share an end line.
        // Run along the pass direction to the point beside the next start,
        // then turn there; this keeps the junction an arc instead of a pivot.
        const double heading = std::atan2(end.GetY() - start.GetY(),
                                          end.GetX() - start.GetX());
        const F2CPoint direction(std::cos(heading), std::sin(heading));
        const F2CPoint offset = previous_end - start;
        const double along = offset.GetX() * direction.GetX() +
                             offset.GetY() * direction.GetY();
        const F2CPoint staging(start.GetX() + direction.GetX() * along,
                               start.GetY() + direction.GetY() * along);
        if (previous_end.Distance(staging) < 0.9 &&
            free.IsPointIn(staging) &&
            staging.Distance(start) <= 0.9) {
          turn = F2CPath();
          AddLine(turn, previous_end, staging,
                  f2c::types::PathSectionType::TURN, velocity);
          turned = AddUTurn(turn, staging, start, free, velocity);
        }
      }
      if (turned) {
        path += turn;
        ++*smooth_turns;
      } else if (previous_end.Distance(start) > 0.9) {
        // A change of strip: follow the room around a concave boundary instead
        // of cutting a straight line through the wall.
        AddTraversableLink(path, previous_end, start, free_space, velocity);
        ++*link_turns;
      } else {
        // No room for the arc: rotate in place instead of leaving the room.
        std::fprintf(stderr,
                     "pivot %.2f,%.2f -> %.2f,%.2f chord %.2f\n",
                     previous_end.GetX(), previous_end.GetY(), start.GetX(),
                     start.GetY(), previous_end.Distance(start));
        AddLine(path, previous_end, previous_end,
                f2c::types::PathSectionType::TURN, velocity);
        path.AddState(previous_end, std::atan2(start.GetY() - previous_end.GetY(),
                                               start.GetX() - previous_end.GetX()),
                      0.0, f2c::types::PathDirection::FORWARD,
                      f2c::types::PathSectionType::TURN, velocity);
        AddLine(path, previous_end, start, f2c::types::PathSectionType::TURN,
                velocity);
        ++*tight_turns;
      }
    }
    AddLine(path, start, end, f2c::types::PathSectionType::SWATH, velocity);
    previous_end = end;
    have_previous = true;
  }
  return path;
}

}  // namespace


namespace {

/** @brief Coverage generated for a single room. */
struct RoomCoverage {
  F2CPath path;
  F2CSwaths swaths;
  double coverage{0.0};
  int swath_count{0};
  int arc_turns{0};
  int pivot_turns{0};
  int link_turns{0};
};

/** @brief Straight connector between two planned pieces inside one room. */
F2CPath RoomLink(const F2CPoint& from, double from_angle, const F2CPoint& to,
                 double to_angle, const Configuration& config) {
  F2CPath link;
  const double length = from.Distance(to);
  if (length < 1e-6) {
    return link;
  }
  const double heading = std::atan2(to.GetY() - from.GetY(),
                                    to.GetX() - from.GetX());
  link.AddState(from, heading, length, f2c::types::PathDirection::FORWARD,
                f2c::types::PathSectionType::TURN, config.turn_speed);
  link.AddState(to, heading, 0.0, f2c::types::PathDirection::FORWARD,
                f2c::types::PathSectionType::TURN, config.turn_speed);
  (void)from_angle;
  (void)to_angle;
  return link;
}

RoomCoverage PlanRoom(const F2CCells& cells, const Configuration& config,
                      const proto::CoverageRequest& goal, bool headland) {
  RoomCoverage output;
  if (cells.IsEmpty() || cells.Area() < 1e-3) {
    return output;
  }
  HeadlandGenerator headlands(config);
  proto::HeadlandMode band_mode = goal.headland_mode();
  if (Unknown(band_mode.mode())) {
    band_mode.set_mode("CONSTANT");
  }
  if (!band_mode.has_width()) {
    band_mode.set_width(config.headland_width);
  }
  // The reference sweep runs each pass from wall to wall, so the swaths are
  // generated on the full room polygon instead of an inset headland band.
  bool band = false;
  F2CCells core = cells.Clone();
  (void)headland;
  (void)band_mode;
  SwathGenerator swath_generator(config);
  auto grouped = swath_generator.GenerateSwathsByCells(core, goal.swath_mode());
  if (grouped.SizeTotal() == 0) {
    return output;
  }
  RouteGenerator route_generator(config);
  F2CRoute route =
      route_generator.GenerateRoute(core, core, grouped, goal.route_mode(), {});
  route.SetField(core);
  F2CSwaths ordered = Flatten(route);
  if (ordered.size() != grouped.SizeTotal()) {
    return output;
  }
  ordered = OrderSweep(ordered);

  int smooth = 0;
  int tight = 0;
  int links = 0;
  F2CPath path = BuildSweepPath(ordered, cells, config.cruise_speed, &smooth,
                                &tight, &links);
  if (path.size() < 2) {
    return output;
  }
  output.arc_turns = smooth;
  output.pivot_turns = tight;
  output.link_turns = links;
  if (tight > 0) {
    std::fprintf(stderr, "room sweep: %d arc turns, %d in-place turns\n", smooth,
                 tight);
  }

  // Sweep the wall band with concentric passes so the room is covered up to
  // the walls while the interior keeps its arc turns.
  F2CPath band_path;
  if (band) {
    PathGenerator path_generator(config);
    try {
      for (const auto& band_cells : headlands.GenerateHeadlandSwaths(
               cells, config.operation_width, band_mode)) {
        F2CPath loop = path_generator.GeneratePerimeter(
            band_cells, cells, path.AtStart(), goal.path_mode());
        if (loop.size() > 1) {
          band_path += loop;
        }
      }
    } catch (const std::exception&) {
      band_path = F2CPath();
    }
  }
  const bool perimeter = band_path.size() > 1;
  F2CPath total;
  if (perimeter) {
    total += band_path;
    total += RoomLink(total.AtEnd(), total.back().angle, path.AtStart(),
                      path[0].angle, config);
  }
  total += path;
  output.path = total;
  output.swaths = ordered;
  output.swath_count = static_cast<int>(ordered.size());
  const double core_coverage =
      f2c::obj::FieldCoverage().ComputeCost(core, ordered);
  const double room_area = cells.Area();
  const double band_area = std::max(0.0, room_area - core.Area());
  output.coverage =
      room_area > 1e-9
          ? (core_coverage * core.Area() + (perimeter ? band_area : 0.0)) /
                room_area
          : core_coverage;
  return output;
}

/**
 * @brief Plan every room of an occupancy map and concatenate one global path.
 *
 * Each room is planned independently so that turn connections never have to
 * cross a wall, then the room paths are joined in nearest-neighbour/2-opt
 * order. The result carries per-room polygons, paths and coverage plus the
 * stitched global path consumed by the visualizer.
 */
ComputeAction::Result PlanMapRegions(
    const Configuration& config, const proto::CoverageRequest& goal,
    const F2CField& master, const F2CCells& field, const std::string& frame,
    const CancelChecker& cancel,
    const std::function<void(const std::string&, double)>& stage) {
  ComputeAction::Result result;
  const bool headland =
      Enabled(goal.has_generate_headland(), goal.generate_headland());
  const double radius =
      goal.has_robot_radius() ? goal.robot_radius() : config.robot_radius;
  MapConnector::Options connector_options;
  if (goal.map().occupied_threshold() > 0) {
    connector_options.occupied_threshold = goal.map().occupied_threshold();
  }
  connector_options.clearance = std::max(0.0, radius);
  const MapConnector connector(goal.map(), connector_options);
  if (!connector.Valid()) {
    throw RequestException("Occupancy map has no traversable free space");
  }
  std::vector<F2CCells> rooms = RegionSegmenter::MapSplit(goal.map(), {});
  if (rooms.empty()) {
    throw RequestException("Map split found no rooms");
  }

  // One anchor per room: the traversable cell closest to the room centroid.
  // Room-to-room costs are navigable distances between anchors, so the tour
  // never assumes a straight line through a wall.
  std::vector<std::pair<double, double>> anchors;
  anchors.reserve(rooms.size());
  for (const auto& cells : rooms) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    int count = 0;
    if (!cells.IsEmpty()) {
      for (const auto& point : cells.GetCellBorder(0)) {
        sum_x += point.GetX();
        sum_y += point.GetY();
        ++count;
      }
    }
    const std::pair<double, double> centroid{
        count > 0 ? sum_x / count : 0.0, count > 0 ? sum_y / count : 0.0};
    anchors.push_back(connector.NearestFree(centroid));
  }

  const bool have_start = goal.has_robot_pose();
  const std::pair<double, double> start =
      have_start
          ? connector.NearestFree(
                {goal.robot_pose().axis1(), goal.robot_pose().axis2()})
          : anchors.front();
  const double kUnreachable = std::numeric_limits<double>::max() / 4.0;
  std::vector<std::vector<double>> costs(
      rooms.size(), std::vector<double>(rooms.size(), kUnreachable));
  std::vector<double> from_start(rooms.size(), kUnreachable);
  for (std::size_t from = 0; from < rooms.size(); ++from) {
    CheckCanceled(cancel);
    for (std::size_t to = 0; to < rooms.size(); ++to) {
      if (from == to) {
        costs[from][to] = 0.0;
      } else {
        costs[from][to] = connector.Distance(anchors[from], anchors[to]);
      }
    }
    from_start[from] =
        have_start ? connector.Distance(start, anchors[from]) : (from == 0 ? 0.0
                                                                        : kUnreachable);
  }

  // Rooms are grouped by navigable connectivity. Only the group reachable
  // from the robot takes part in the tour; sealed rooms are still reported
  // with their own plan so the operator can see what is left behind a wall.
  std::vector<int> group(rooms.size(), -1);
  int groups = 0;
  for (std::size_t seed = 0; seed < rooms.size(); ++seed) {
    if (group[seed] >= 0) {
      continue;
    }
    std::vector<std::size_t> stack{seed};
    group[seed] = groups;
    while (!stack.empty()) {
      const std::size_t index = stack.back();
      stack.pop_back();
      for (std::size_t other = 0; other < rooms.size(); ++other) {
        if (group[other] < 0 && costs[index][other] < kUnreachable) {
          group[other] = groups;
          stack.push_back(other);
        }
      }
    }
    ++groups;
  }
  int active_group = group.front();
  if (have_start) {
    double best = kUnreachable;
    for (std::size_t index = 0; index < rooms.size(); ++index) {
      if (from_start[index] < best) {
        best = from_start[index];
        active_group = group[index];
      }
    }
  }

  std::vector<std::size_t> visit;
  if (active_group < 0) {
    for (std::size_t index = 0; index < rooms.size(); ++index) {
      visit.push_back(index);
    }
  } else {
    std::vector<std::size_t> active;
    std::vector<std::size_t> sealed;
    for (std::size_t index = 0; index < rooms.size(); ++index) {
      (group[index] == active_group ? active : sealed).push_back(index);
    }
    std::vector<std::vector<double>> local(
        active.size(), std::vector<double>(active.size(), 0.0));
    std::vector<double> local_start(active.size(), 0.0);
    for (std::size_t from = 0; from < active.size(); ++from) {
      for (std::size_t to = 0; to < active.size(); ++to) {
        local[from][to] = costs[active[from]][active[to]];
      }
      local_start[from] =
          have_start ? from_start[active[from]] : (active[from] == 0 ? 0.0
                                                                   : kUnreachable);
    }
    const auto ordered = RegionOrderer::Order(local, local_start);
    for (const std::size_t index : ordered.order) {
      visit.push_back(active[index]);
    }
    if (!sealed.empty()) {
      result.add_warnings(std::to_string(sealed.size()) + " of " +
                          std::to_string(rooms.size()) +
                          " room(s) are sealed off from the robot; open a door "
                          "to include them in the tour");
    }
    for (const std::size_t index : sealed) {
      visit.push_back(index);
    }
  }

  NavPath* nav = result.mutable_nav_path();
  nav->mutable_header()->set_frame_id(frame);
  auto* components = result.mutable_coverage_path();
  components->set_frame_id(frame);
  components->set_contains_turns(true);
  components->set_swaths_ordered(true);

  auto emit_pose = [&](double x, double y, double yaw) {
    auto* pose = nav->add_poses();
    pose->mutable_header()->set_frame_id(frame);
    auto* position = pose->mutable_pose()->mutable_position();
    position->set_x(x);
    position->set_y(y);
    auto* orientation = pose->mutable_pose()->mutable_orientation();
    orientation->set_z(std::sin(yaw * 0.5));
    orientation->set_w(std::cos(yaw * 0.5));
  };
  auto emit_leg = [&](const std::pair<double, double>& from,
                      const std::pair<double, double>& to) {
    const double distance =
        std::hypot(to.first - from.first, to.second - from.second);
    if (distance < 1e-6) {
      return;
    }
    const double yaw =
        std::atan2(to.second - from.second, to.first - from.first);
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / 0.15)));
    emit_pose(from.first, from.second, yaw);
    for (int step = 1; step < steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      emit_pose(from.first + (to.first - from.first) * ratio,
                from.second + (to.second - from.second) * ratio, yaw);
    }
  };

  double weighted_coverage = 0.0;
  double total_area = 0.0;
  double travel = 0.0;
  double task_time = 0.0;
  std::pair<double, double> cursor = start;
  bool have_cursor = have_start;
  int index = 0;
  int connected = 0;
  int arc_turns = 0;
  int pivot_turns = 0;
  int link_turns = 0;
  int planned = 0;
  for (const std::size_t room_index : visit) {
    CheckCanceled(cancel);
    const bool reachable = active_group >= 0 && group[room_index] == active_group;
    stage("room " + std::to_string(index + 1) + "/" +
              std::to_string(visit.size()),
          0.15 + 0.75 * static_cast<double>(index) / visit.size());
    const F2CCells& cells = rooms[room_index];
    auto* region = result.add_regions();
    region->set_id(index);
    region->set_order(index + 1);
    region->set_name("Region " + std::to_string(index + 1));
    region->set_color_index(index % 8);
    const double area = RegionSegmenter::Area(cells);
    region->set_area(area);
    for (const auto& cell : cells) {
      for (const auto& ring : cell) {
        const auto points = CleanRing(ring);
        if (points.size() < 3) {
          continue;
        }
        auto* polygon = region->add_polygons();
        for (const auto& point : points) {
          auto* coordinate = polygon->add_coordinates();
          coordinate->set_axis1(point.first);
          coordinate->set_axis2(point.second);
        }
      }
    }

    RoomCoverage room;
    try {
      room = PlanRoom(cells, config, goal, headland);
    } catch (const std::exception& error) {
      result.add_warnings("Region " + std::to_string(index + 1) + ": " +
                          error.what());
      // A headland band that cannot host a turn still leaves the room
      // coverable: retry with a plain boustrophedon sweep.
      try {
        room = PlanRoom(cells, config, goal, false);
      } catch (const std::exception& retry) {
        // Household rooms are tight: fall back to a turning radius suited to
        // the robot footprint before reporting the room as unplannable.
        Configuration tight = config;
        tight.min_turning_radius =
            std::max(0.15, std::min(config.min_turning_radius,
                                    radius > 0.0 ? radius : 0.4));
        try {
          room = PlanRoom(cells, tight, goal, false);
        } catch (const std::exception& last) {
          result.add_warnings("Region " + std::to_string(index + 1) +
                              " tight-radius retry failed: " + last.what());
        }
        result.add_warnings("Region " + std::to_string(index + 1) +
                            " retry failed: " + retry.what());
      }
    }
    ComputeAction::Result planned_room;
    F2CField room_field(cells);
    room_field.SetRefPoint(F2CPoint(0.0, 0.0));
    room_field.SetCRS(frame);
    if (room.swaths.size() > 0) {
      ExportSwaths(room.swaths, room_field, frame, true, true,
                   *planned_room.mutable_coverage_path());
    }
    if (room.path.size() > 1) {
      ExportPath(room.path, room_field, frame, true,
                 config.turn_point_distance, planned_room);
    }
    for (const auto& swath : planned_room.coverage_path().swaths()) {
      *components->add_swaths() = swath;
    }
    for (const auto& state : planned_room.coverage_path().states()) {
      *components->add_states() = state;
    }
    *region->mutable_nav_path() = planned_room.nav_path();
    for (const auto& state : planned_room.coverage_path().states()) {
      *region->add_states() = state;
    }
    arc_turns += room.arc_turns;
    pivot_turns += room.pivot_turns;
    link_turns += room.link_turns;
    region->set_coverage(reachable ? room.coverage : 0.0);
    region->set_swath_count(reachable ? room.swath_count : 0);
    if (reachable && planned_room.nav_path().poses_size() > 0) {
      const auto& entry_pose = planned_room.nav_path().poses(0).pose().position();
      const std::pair<double, double> entry{entry_pose.x(), entry_pose.y()};
      if (have_cursor) {
        const auto link = connector.Connect(cursor, entry);
        if (!link.points.empty()) {
          for (std::size_t point = 1; point < link.points.size(); ++point) {
            emit_leg(link.points[point - 1], link.points[point]);
          }
          travel += link.distance;
          ++connected;
        }
      }
      for (const auto& pose : planned_room.nav_path().poses()) {
        *nav->add_poses() = pose;
      }
      const auto& last = nav->poses(nav->poses_size() - 1).pose().position();
      cursor = {last.x(), last.y()};
      have_cursor = true;
      ++planned;
    }
    if (area > 1e-9) {
      weighted_coverage += (reachable ? room.coverage : 0.0) * area;
    }
    total_area += area;
    task_time += room.path.GetTaskTime();
    ++index;
  }
  result.set_core_coverage(total_area > 0.0 ? weighted_coverage / total_area
                                            : 0.0);
  result.set_field_wkt((field + master.GetRefPoint()).ExportToWkt());
  result.set_core_wkt(result.field_wkt());
  result.set_travel_distance(travel);
  result.set_task_time(std::isfinite(task_time) ? task_time : 0.0);
  result.add_warnings("sweep turns: " + std::to_string(arc_turns) +
                      " arc, " + std::to_string(pivot_turns) + " pivot, " +
                      std::to_string(link_turns) + " link");
  result.add_warnings(std::to_string(planned) + " room(s) planned through " +
                      std::to_string(connected) +
                      " traversable connection(s)");
  return result;
}

}  // namespace
CoverageServer::CoverageServer(Configuration c) : config_(std::move(c)) {
  RobotParams validate(config_);
}
void ExportSwaths(const F2CSwaths& swaths, const F2CField& field,
                  const std::string& frame, bool cartesian, bool ordered,
                  proto::PathComponents& output) {
  output.set_frame_id(frame);
  output.set_swaths_ordered(ordered);
  for (const auto& swath : swaths) {
    auto* msg = output.add_swaths();
    msg->set_id(swath.GetId());
    msg->set_width(swath.GetWidth());
    msg->mutable_path()->mutable_header()->set_frame_id(frame);
    for (const auto& p : swath.GetPath())
      Pose(*msg->mutable_path(), OutputPoint(p, field, cartesian),
           swath.GetInAngle(), frame);
  }
}
void ExportPath(const F2CPath& path, const F2CField& field,
                const std::string& frame, bool cartesian, double step,
                ComputeAction::Result& output) {
  auto* components = output.mutable_coverage_path();
  components->set_frame_id(frame);
  components->set_contains_turns(true);
  components->set_swaths_ordered(true);
  auto* nav = output.mutable_nav_path();
  nav->mutable_header()->set_frame_id(frame);
  NavPath* current_turn = nullptr;
  for (const auto& state : path) {
    const auto p = OutputPoint(state.point, field, cartesian);
    auto* msg = components->add_states();
    msg->set_x(p.GetX());
    msg->set_y(p.GetY());
    msg->set_z(p.GetZ());
    msg->set_angle(state.angle);
    msg->set_len(state.len);
    msg->set_direction(static_cast<int>(state.dir));
    msg->set_type(static_cast<int>(state.type));
    msg->set_velocity(state.velocity);
    if (state.type != f2c::types::PathSectionType::SWATH) {
      if (!current_turn) {
        current_turn = components->add_turns();
        current_turn->mutable_header()->set_frame_id(frame);
      }
      Pose(*current_turn, p, state.angle, frame);
    } else
      current_turn = nullptr;
    const unsigned count =
        std::max(1u, static_cast<unsigned>(std::ceil(state.len / step)));
    if (nav->poses_size() + count > 1000000)
      throw RequestException("Coverage path exceeds one million sampled poses");
    for (unsigned i = 0; i < count; ++i) {
      const auto sample = state.point.GetPointFromAngle(
          state.angle, state.len * static_cast<int>(state.dir) * i / count);
      Pose(*nav, OutputPoint(sample, field, cartesian), state.angle, frame);
      components->add_velocities(state.velocity);
      components->add_is_backward(state.dir ==
                                  f2c::types::PathDirection::BACKWARD);
    }
  }
  if (path.size()) {
    Pose(*nav, OutputPoint(path.AtEnd(), field, cartesian), path.back().angle,
         frame);
    components->add_velocities(0);
    components->add_is_backward(path.back().dir ==
                                f2c::types::PathDirection::BACKWARD);
  }
}
ComputeAction::Result CoverageServer::Compute(
    const proto::CoverageRequest& goal, const CancelChecker& cancel,
    const Progress& feedback) const {
  const auto began = std::chrono::steady_clock::now();
  ComputeAction::Result result;
  auto stage = [&](const std::string& name, double progress) {
    CheckCanceled(cancel);
    if (feedback) feedback(name, progress);
  };
  try {
    Configuration effective = config_;
    if (goal.has_robot_radius()) {
      if (!std::isfinite(goal.robot_radius()) || goal.robot_radius() <= 0.0)
        throw RequestException("robot_radius must be a positive metre value");
      effective.robot_radius = goal.robot_radius();
      effective.operation_width = 2.0 * goal.robot_radius();
      RobotParams validate(effective);
    }
    if (goal.has_swath_spacing()) {
      if (!std::isfinite(goal.swath_spacing()) ||
          goal.swath_spacing() <= 0.0) {
        throw RequestException(
            "swath_spacing must be a positive metre value");
      }
      effective.operation_width = goal.swath_spacing();
      RobotParams validate(effective);
    }
    const bool headland =
        Enabled(goal.has_generate_headland(), goal.generate_headland());
    const bool route_enabled =
        Enabled(goal.has_generate_route(), goal.generate_route());
    const bool path_enabled =
        Enabled(goal.has_generate_path(), goal.generate_path());
    const bool cartesian = goal.has_coordinates_in_cartesian_frame()
                               ? goal.coordinates_in_cartesian_frame()
                               : effective.cartesian;
    if (path_enabled && !route_enabled)
      throw RequestException("generate_path requires generate_route");
    if (goal.generate_headland_swaths() && (!headland || !path_enabled))
      throw RequestException(
          "Headland swaths require the complete headland/route/path pipeline");
    if (goal.rows().empty() && !Unknown(goal.row_swath_mode().mode()))
      throw RequestException("Row mode requires annotated rows");
    if (goal.use_start_pose() &&
        (!route_enabled ||
         Upper(RouteGenerator(effective).Resolve(goal.route_mode()).mode()) !=
             "TSP"))
      throw RequestException("start_pose requires TSP route mode");
    stage("field", 0.0);
    F2CField master;
    std::string frame = goal.frame_id().empty() ? "map" : goal.frame_id();
    if (goal.use_gml_file()) {
      if (goal.gml_field().empty())
        throw RequestException("Missing GML filename");
      master = f2c::Parser::ImportFieldGml(goal.gml_field(), true);
      if (!master.GetCRS().empty()) frame = master.GetCRS();
    } else {
      if (goal.polygons().empty())
        throw RequestException("Missing field polygon");
      F2CCell cell;
      for (const auto& polygon : goal.polygons()) {
        if (polygon.coordinates_size() < 3)
          throw RequestException("Ring needs at least three vertices");
        F2CLinearRing ring;
        for (const auto& p : polygon.coordinates()) {
          if (!std::isfinite(p.axis1()) || !std::isfinite(p.axis2()))
            throw RequestException("Nonfinite coordinate");
          ring.AddPoint(p.axis1(), p.axis2());
        }
        ring.CloseRing();
        cell.AddRing(ring);
      }
      Validate(F2CCells(cell));
      master = F2CField(F2CCells(cell));
      master.SetCRS(frame);
    }
    if (!cartesian) {
      if (master.GetCRS() != "EPSG:4326" && master.GetCRS() != "EPSG:4258")
        throw RequestException("GPS requests require EPSG:4326 or EPSG:4258");
      f2c::Transform::TransformToUTM(master);
    }
    auto field = master.GetField().Clone();
    Validate(field);
    // A map-driven request plans every room on its own: one global plan over
    // the bounding rectangle has to cross walls and its turn connections are
    // frequently infeasible. Per-room plans are stitched afterwards.
    if (goal.generate_regions() && goal.has_map()) {
      result = PlanMapRegions(effective, goal, master, field, frame, cancel,
                              stage);
      stage("complete", 1.0);
      result.set_error_code(proto::NONE);
      result.set_planning_time(
          std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
              .count());
      return result;
    }
    stage("headland", 0.1);
    HeadlandGenerator hg(effective);
    auto core = headland ? hg.GenerateHeadlands(field, goal.headland_mode())
                         : field.Clone();
    F2CCells travel = core.Clone(), swath_cells = core.Clone();
    if (goal.generate_decomp()) {
      stage("decomposition", 0.2);
      travel = DecompGenerator(effective).Decompose(core, goal.decomp_mode());
      swath_cells = headland ? hg.GenerateHeadlandsBetweenCells(
                                   travel, goal.route_headland_width() > 0
                                               ? goal.route_headland_width()
                                               : effective.operation_width)
                             : travel.Clone();
    }
    Validate(swath_cells);
    stage("swaths", 0.3);
    F2CSwathsByCells grouped;
    if (!goal.rows().empty()) {
      if (goal.generate_decomp() || !cartesian)
        throw RequestException(
            "Annotated row input currently requires one Cartesian field");
      auto rows = GenerateRowSwaths(goal, master.GetRefPoint());
      F2CSwaths clipped;
      for (const auto& row : rows) {
        for (const auto& line : field.GetLinesInside(row.GetPath()))
          if (line.size() >= 2 && line.Length() > 1e-8)
            clipped.emplace_back(line.Clone(), row.GetWidth(), row.GetId());
      }
      grouped.emplace_back(clipped);
      swath_cells = field.Clone();
      travel = field.Clone();
      core = field.Clone();
    } else
      grouped = SwathGenerator(effective).GenerateSwathsByCells(
          swath_cells, goal.swath_mode());
    if (grouped.SizeTotal() == 0 || grouped.SizeTotal() > 1000)
      throw RequestException("Coverage needs 1..1000 swaths");
    F2CSwaths swaths;
    for (const auto& group : grouped) swaths.Append(group);
    F2CPath path;
    F2CRoute route;
    if (route_enabled) {
      stage("route", 0.55);
      std::optional<F2CPoint> start;
      if (goal.use_start_pose()) {
        F2CPoint p(goal.start_pose().axis1(), goal.start_pose().axis2());
        if (!cartesian)
          p = f2c::Transform::transform(p, master.GetPrevCRS(),
                                        master.GetCRS());
        start = p - master.GetRefPoint();
        if (!field.IsPointIn(*start))
          throw RequestException("start_pose must lie in traversable field");
      }
      route = RouteGenerator(effective).GenerateRoute(
          travel, swath_cells, grouped, goal.route_mode(), start);
      route.SetField(field);
      swaths = Flatten(route);
      if (swaths.size() != grouped.SizeTotal())
        throw std::runtime_error("Route lost generated swaths");
      if (path_enabled) {
        stage("path", 0.7);
        PathGenerator pp(effective);
        path = pp.GeneratePath(route, goal.path_mode());
        if (goal.generate_headland_swaths()) {
          stage("headland_swaths", 0.8);
          F2CPath perimeter;
          if (goal.headland_full_coverage()) {
            for (const auto& ring : hg.GenerateHeadlandSwaths(
                     field, effective.operation_width, goal.headland_mode())) {
              auto next = pp.GeneratePerimeter(ring, field, path.AtStart(), goal.path_mode());
              if (perimeter.size() > 0 && next.size() > 0) {
                RobotParams ring_robot(effective);
                auto ring_turn = pp.CreateCurve(goal.path_mode());
                perimeter += f2c::pp::PathPlanning::PlanPathForConnection(
                    ring_robot.GetRobot(), perimeter.AtEnd(), perimeter.back().angle,
                    F2CMultiPoint(), next.AtStart(), next[0].angle, *ring_turn, &field);
              }
              perimeter += next;
            }
          } else
            perimeter = pp.GeneratePerimeter(core, field, path.AtStart(),
                                             goal.path_mode());
          RobotParams params(effective);
          auto turn = pp.CreateCurve(goal.path_mode());
          auto connection = [&](const F2CPath& a, const F2CPath& b) {
            return f2c::pp::PathPlanning::PlanPathForConnection(
                params.GetRobot(), a.AtEnd(), a.back().angle, F2CMultiPoint(),
                b.AtStart(), b[0].angle, *turn, &field);
          };
          F2CPath combined;
          if (goal.headland_first()) {
            combined = perimeter;
            combined += connection(perimeter, path);
            combined += path;
          } else {
            combined = path;
            combined += connection(path, perimeter);
            combined += perimeter;
          }
          path = std::move(combined);
        }
        stage("validation", 0.9);
        RobotParams robot(effective);
        f2c::pp::detail::FreeSpace free(field);
        if (!f2c::pp::detail::ValidPath(path, robot.GetRobot(), &free)) {
          for (size_t i = 0; i < path.size(); ++i) {
            F2CPath pair;
            pair.AddState(path[i]);
            if (i + 1 < path.size()) pair.AddState(path[i + 1]);
            if (!f2c::pp::detail::ValidPath(pair, robot.GetRobot(), &free))
              throw std::runtime_error(
                  "Final path constraint failure at state " +
                  std::to_string(i) + ": " + pair.SerializePath(9));
          }
          throw std::runtime_error(
              "Final path violates field, hole, join or curvature constraints");
        }
        for (const auto& state : path)
          if (!std::isfinite(state.velocity) || state.velocity < 0)
            throw std::runtime_error("Invalid path velocity");
        ExportPath(path, master, frame, cartesian, effective.turn_point_distance,
                   result);
        const double task_time = path.GetTaskTime();
        result.set_task_time(std::isfinite(task_time) ? task_time : 0);
      }
    }
    ExportSwaths(swaths, master, frame, cartesian, route_enabled,
                 *result.mutable_coverage_path());
    result.set_core_coverage(
        f2c::obj::FieldCoverage().ComputeCost(core, swaths));
    result.set_field_wkt((field + master.GetRefPoint()).ExportToWkt());
    result.set_core_wkt((core + master.GetRefPoint()).ExportToWkt());
    if (!cartesian)
      result.add_warnings(
          "GPS output poses use geographic units; execution requires a "
          "Cartesian request");
    if (goal.generate_regions()) {
      const F2CPoint reference = master.GetRefPoint();
      const bool map_based = goal.has_map();
      RegionSegmenter::Options options;
      options.columns = goal.region_columns() > 0 ? goal.region_columns() : 2;
      options.rows = goal.region_rows() > 0 ? goal.region_rows() : 2;
      std::vector<F2CCells> regions =
          map_based ? RegionSegmenter::MapSplit(goal.map(), RegionSegmenter::MapOptions{})
                    : RegionSegmenter::GridSplit(field, options);
      std::vector<std::pair<double, double>> centroids;
      for (const auto& cells : regions) {
        if (cells.IsEmpty()) {
          centroids.emplace_back(0.0, 0.0);
          continue;
        }
        double sum_x = 0.0;
        double sum_y = 0.0;
        int count = 0;
        for (const auto& point : cells.GetCellBorder(0)) {
          sum_x += point.GetX() + (map_based ? 0.0 : reference.GetX());
          sum_y += point.GetY() + (map_based ? 0.0 : reference.GetY());
          ++count;
        }
        centroids.emplace_back(count > 0 ? sum_x / count : 0.0,
                               count > 0 ? sum_y / count : 0.0);
      }
      const double start_x = goal.use_start_pose() ? goal.start_pose().axis1() : 0.0;
      const double start_y = goal.use_start_pose() ? goal.start_pose().axis2() : 0.0;
      const auto ordered = RegionOrderer::Order(centroids, start_x, start_y);
      std::vector<F2CCells> ordered_regions;
      ordered_regions.reserve(regions.size());
      for (const std::size_t index : ordered.order) {
        ordered_regions.push_back(std::move(regions[index]));
      }
      regions = std::move(ordered_regions);
      result.set_travel_distance(ordered.distance);
      const auto shift_cells = [](const F2CCells& cells, double dx,
                                  double dy) {
        F2CCells output;
        for (const auto& cell : cells) {
          F2CCell shifted;
          for (const auto& ring : cell) {
            const auto points = CleanRing(ring);
            if (points.size() < 3) {
              continue;
            }
            F2CLinearRing result_ring;
            for (const auto& point : points) {
              result_ring.AddPoint(point.first + dx, point.second + dy);
            }
            result_ring.CloseRing();
            shifted.AddRing(result_ring);
          }
          if (shifted.Area() > 1e-6) {
            output.AddGeometry(shifted);
          }
        }
        return output;
      };
      int region_index = 0;
      for (const auto& cells : regions) {
        const F2CCells local =
            shift_cells(cells, map_based ? -reference.GetX() : 0.0,
                        map_based ? -reference.GetY() : 0.0);
        const F2CCells display =
            shift_cells(cells, map_based ? 0.0 : reference.GetX(),
                        map_based ? 0.0 : reference.GetY());
        if (local.IsEmpty() || display.IsEmpty()) {
          continue;
        }
        auto* region = result.add_regions();
        region->set_id(region_index);
        region->set_order(region_index + 1);
        region->set_name("Region " + std::to_string(region_index + 1));
        region->set_color_index(region_index % 8);
        const double area = RegionSegmenter::Area(display);
        region->set_area(area);
        for (const auto& region_cell : display) {
          for (const auto& ring : region_cell) {
            const auto points = CleanRing(ring);
            if (points.size() < 3) {
              continue;
            }
            auto* polygon = region->add_polygons();
            for (const auto& point : points) {
              auto* coordinate = polygon->add_coordinates();
              coordinate->set_axis1(point.first);
              coordinate->set_axis2(point.second);
            }
          }
        }
        region->set_coverage(
            f2c::obj::FieldCoverage().ComputeCost(local, swaths));
        int swath_count = 0;
        for (const auto& swath : swaths) {
          if (local.IsPointIn(swath.GetPath().StartPoint())) {
            ++swath_count;
          }
        }
        region->set_swath_count(swath_count);
        for (const auto& state : result.coverage_path().states()) {
          if (local.IsPointIn(F2CPoint(state.x(), state.y()))) {
            *region->add_states() = state;
          }
        }
        ++region_index;
      }
    }
    stage("complete", 1.0);
    result.set_error_code(proto::NONE);
  } catch (const CanceledException& e) {
    result.Clear();
    result.set_error_code(proto::CANCELED);
    result.set_error_msg(e.what());
  } catch (const CoverageException& e) {
    result.Clear();
    result.set_error_code(proto::INVALID_MODE_SET);
    result.set_error_msg(e.what());
  } catch (const std::invalid_argument& e) {
    result.Clear();
    result.set_error_code(proto::INVALID_REQUEST);
    result.set_error_msg(e.what());
  } catch (const std::exception& e) {
    result.Clear();
    result.set_error_code(proto::INTERNAL_F2C_ERROR);
    result.set_error_msg(e.what());
  }
  result.set_planning_time(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
          .count());
  return result;
}
}  // namespace autonomy::task::coverage
