// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file path_generator.cpp
 * @brief Native complete-coverage path generator implementation. */
#include "autonomy/task/navigation/coverage/path_generator.hpp"

#include <cmath>
#include <vector>

#include "autonomy/task/navigation/coverage/generator_utils.hpp"
#include "autonomy/task/navigation/coverage/robot_params.hpp"
namespace autonomy::task::coverage {
namespace {

// Headland rings come from a raster buffer and contain many collinear points
// only a grid cell apart.  Treating each one as an independent perimeter edge
// makes every edge shorter than the turning-radius trim, which silently drops
// the whole wall band.  Collapse those points before building the route.
std::vector<F2CPoint> SimplifyRing(const F2CLinearRing& ring) {
  std::vector<F2CPoint> points;
  for (size_t i = 0; i < ring.size(); ++i) {
    const F2CPoint point = ring.GetGeometry(i);
    if (points.empty() || points.back().Distance(point) > 1e-4) {
      points.push_back(point);
    }
  }
  if (points.size() > 1 && points.front().Distance(points.back()) <= 1e-4) {
    points.pop_back();
  }
  if (points.size() < 3) {
    return points;
  }

  bool changed = true;
  while (changed && points.size() >= 3) {
    changed = false;
    std::vector<F2CPoint> kept;
    const size_t count = points.size();
    for (size_t i = 0; i < count; ++i) {
      const F2CPoint& previous = points[(i + count - 1) % count];
      const F2CPoint& current = points[i];
      const F2CPoint& next = points[(i + 1) % count];
      const double ax = current.GetX() - previous.GetX();
      const double ay = current.GetY() - previous.GetY();
      const double bx = next.GetX() - current.GetX();
      const double by = next.GetY() - current.GetY();
      const double span = std::max(1e-9, previous.Distance(next));
      const double distance = std::abs(ax * by - ay * bx) / span;
      const double forward = ax * bx + ay * by;
      if (distance < 1e-3 && forward >= -1e-6) {
        changed = true;
        continue;
      }
      kept.push_back(current);
    }
    if (kept.size() < 3) {
      break;
    }
    points.swap(kept);
  }
  return points;
}

void AppendConnector(F2CPath& path, const F2CPoint& target, double target_angle,
                     const F2CRobot& robot, f2c::pp::TurningBase& turn,
                     const Configuration& config) {
  (void)turn;
  if (path.size() == 0) {
    return;
  }
  const F2CPoint from = path.AtEnd();
  // AppendSwath stores every point except the last one, so the trailing state
  // still carries the full last-segment length.  Seal it with a zero-length
  // state before a sampled connector: otherwise the validation sees a state
  // whose len overshoots the next connector point by a whole swath segment.
  if (std::abs(path.back().len) > 1e-9) {
    path.AddState(from, path.back().angle, 0.0,
                  f2c::types::PathDirection::FORWARD,
                  f2c::types::PathSectionType::TURN, config.turn_speed);
  }
  if (from.Distance(target) <= 1e-6) {
    return;
  }
  const double start_angle = path.back().angle;
  const double d1x = std::cos(start_angle), d1y = std::sin(start_angle);
  const double d2x = std::cos(target_angle), d2y = std::sin(target_angle);
  const double dx = target.GetX() - from.GetX();
  const double dy = target.GetY() - from.GetY();
  const double denominator = d1x * d2y - d1y * d2x;
  bool have_corner = false;
  F2CPoint corner;
  if (std::abs(denominator) > 1e-4) {
    const double along = (dx * d2y - dy * d2x) / denominator;
    corner = F2CPoint(from.GetX() + d1x * along, from.GetY() + d1y * along);
    const double behind =
        (corner.GetX() - target.GetX()) * d2x +
        (corner.GetY() - target.GetY()) * d2y;
    have_corner = along > 1e-3 && behind <= 1e-3;
  }
  // A quadratic Bezier that is tangent to both edges rounds the corner.  Its
  // endpoint curvature is 1/arm, so a short arm (a short wall segment or a
  // narrow headland band) would break the robot's turning radius.  Sample the
  // curve first and only keep it when the sampled curvature stays feasible.
  if (have_corner) {
    const double span = from.Distance(corner) + corner.Distance(target);
    const int steps = std::clamp(
        static_cast<int>(std::ceil(span / config.turn_point_distance)), 2, 256);
    std::vector<F2CPoint> samples;
    samples.reserve(static_cast<std::size_t>(steps));
    for (int i = 1; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      const double u = 1.0 - t;
      samples.emplace_back(
          u * u * from.GetX() + 2.0 * u * t * corner.GetX() +
              t * t * target.GetX(),
          u * u * from.GetY() + 2.0 * u * t * corner.GetY() +
              t * t * target.GetY());
    }
    const double limit =
        1.01 / std::max(1e-6, robot.GetMinTurningRadius());
    double worst = 0.0;
    double previous_angle = start_angle;
    F2CPoint previous = from;
    for (const auto& point : samples) {
      const double length = previous.Distance(point);
      if (length > 1e-9) {
        const double angle = (point - previous).GetAngleFromPoint();
        worst = std::max(worst,
                         F2CPoint::GetAngleDiffAbs(previous_angle, angle) / length);
        previous_angle = angle;
      }
      previous = point;
    }
    if (worst <= 0.98 * limit) {
      previous = from;
      previous_angle = start_angle;
      for (const auto& point : samples) {
        const double length = previous.Distance(point);
        const double angle = (point - previous).GetAngleFromPoint();
        path.AddState(previous, angle, length,
                      f2c::types::PathDirection::FORWARD,
                      f2c::types::PathSectionType::TURN, config.turn_speed);
        previous = point;
        previous_angle = angle;
      }
      (void)previous_angle;
      path.AddState(target, target_angle, 0.0,
                    f2c::types::PathDirection::FORWARD,
                    f2c::types::PathSectionType::TURN, config.turn_speed);
      return;
    }
  }
  // Tight corner: the band cannot host a radius-limited arc, so pivot in place
  // at the wall corner.  Zero-length states keep the curvature check valid on
  // both sides of the rotation.
  const F2CPoint pivot = have_corner ? corner : target;
  if (pivot.Distance(from) > 1e-6) {
    const double heading = (pivot - from).GetAngleFromPoint();
    if (std::abs(F2CPoint::GetAngleDiffAbs(start_angle, heading)) > 1e-9) {
      path.AddState(from, heading, 0.0, f2c::types::PathDirection::FORWARD,
                    f2c::types::PathSectionType::TURN, config.turn_speed);
    }
    path.AppendStraight(from, pivot, config.cruise_speed,
                        f2c::types::PathSectionType::TURN);
    path.AddState(pivot, heading, 0.0, f2c::types::PathDirection::FORWARD,
                  f2c::types::PathSectionType::TURN, config.turn_speed);
  }
  if (pivot.Distance(target) > 1e-6) {
    const double heading = (target - pivot).GetAngleFromPoint();
    if (std::abs(F2CPoint::GetAngleDiffAbs(start_angle, heading)) > 1e-9) {
      path.AddState(pivot, heading, 0.0, f2c::types::PathDirection::FORWARD,
                    f2c::types::PathSectionType::TURN, config.turn_speed);
    }
    path.AppendStraight(pivot, target, config.cruise_speed,
                        f2c::types::PathSectionType::TURN);
  }
  path.AddState(target, (target - pivot).GetAngleFromPoint(), 0.0,
                f2c::types::PathDirection::FORWARD,
                f2c::types::PathSectionType::TURN, config.turn_speed);
}

// Fallback for a narrow or highly concave headland ring.  The normal route
// planner refuses to connect two perimeter edges when the collision-free
// bounded-curvature search cannot find a turn inside the thin band.  Walking
// the simplified ring directly preserves the wall coverage and leaves the
// interior sweep to the normal swath planner.
void AppendFallbackRing(F2CPath& path, const F2CSwaths& edges,
                        const F2CRobot& robot, f2c::pp::TurningBase& turn,
                        const Configuration& config) {
  if (edges.size() == 0) {
    return;
  }
  const F2CSwath& first = edges[0];
  AppendConnector(path, first.StartPoint(), first.GetInAngle(), robot, turn,
                  config);
  path.AppendSwath(first, config.cruise_speed);
  for (size_t i = 1; i < edges.size(); ++i) {
    AppendConnector(path, edges[i].StartPoint(), edges[i].GetInAngle(), robot,
                    turn, config);
    path.AppendSwath(edges[i], config.cruise_speed);
  }
  AppendConnector(path, first.StartPoint(), first.GetInAngle(), robot, turn,
                  config);
}

// Last-resort path for an obstacle hole: stay on the simplified hole boundary
// instead of rounding into the hole.  This is only used when the checked
// connector cannot find a bounded-curvature turn around the obstacle.
void AppendBoundaryRing(F2CPath& path, const std::vector<F2CPoint>& ring,
                        const Configuration& config) {
  if (ring.size() < 3) {
    return;
  }
  if (path.size() == 0) {
    path.AppendStraight(ring[0], ring[1], config.cruise_speed,
                        f2c::types::PathSectionType::TURN);
  } else {
    path.AppendStraight(path.AtEnd(), ring[0], config.cruise_speed,
                        f2c::types::PathSectionType::TURN);
  }
  for (size_t i = 1; i < ring.size(); ++i) {
    path.AppendStraight(path.AtEnd(), ring[(i + 1) % ring.size()],
                        config.cruise_speed,
                        f2c::types::PathSectionType::TURN);
  }
}

}  // namespace

std::unique_ptr<f2c::pp::TurningBase> PathGenerator::CreateCurve(
    const proto::PathMode& m) const {
  const bool defaults = Unknown(m.mode()) && Unknown(m.continuity_mode());
  if (!defaults && (Unknown(m.mode()) || Unknown(m.continuity_mode())))
    throw CoverageException("Path override requires mode and continuity_mode");
  const auto type = Upper(defaults ? config_.path_type : m.mode()),
             continuity = Upper(defaults ? config_.path_continuity_type
                                         : m.continuity_mode());
  std::unique_ptr<f2c::pp::TurningBase> curve;
  if (type == "DUBIN" && continuity == "DISCONTINUOUS")
    curve = std::make_unique<f2c::pp::DubinsCurves>();
  else if (type == "DUBIN" && continuity == "CONTINUOUS")
    curve = std::make_unique<f2c::pp::DubinsCurvesCC>();
  else if (type == "REEDS_SHEPP" && continuity == "DISCONTINUOUS")
    curve = std::make_unique<f2c::pp::ReedsSheppCurves>();
  else if (type == "REEDS_SHEPP" && continuity == "CONTINUOUS")
    curve = std::make_unique<f2c::pp::ReedsSheppCurvesHC>();
  else
    throw CoverageException("Invalid path or continuity mode");
  const double step =
      defaults ? config_.turn_point_distance
               : (m.has_turn_point_distance() ? m.turn_point_distance() : 0.1);
  Positive(step, "turn_point_distance");
  if (step < 1e-3 || step > 0.5)
    throw RequestException("turn_point_distance outside [0.001,0.5]");
  curve->SetDiscretization(step);
  return curve;
}
F2CPath PathGenerator::GeneratePath(const F2CRoute& route,
                                    const proto::PathMode& m) const {
  RobotParams robot(config_);
  auto curve = CreateCurve(m);
  return f2c::pp::PathPlanning::PlanPath(robot.GetRobot(), route, *curve);
}
F2CPath PathGenerator::GeneratePerimeter(const F2CCells& rings,
                                         const F2CCells& field,
                                         const F2CPoint& near,
                                         const proto::PathMode& m) const {
  // Preserve upstream perimeter semantics, but round corners through the native
  // checked connector.
  RobotParams params(config_);
  auto& robot = params.GetRobot();
  auto fallback_turn = CreateCurve(m);
  F2CPath path;
  for (const auto& cell : rings)
    for (size_t ring_index = 0; ring_index < cell.size(); ++ring_index) {
      const auto& raw_ring = cell[ring_index];
      const bool hole = ring_index > 0;
      const auto ring = SimplifyRing(raw_ring);
      if (ring.size() < 3) continue;
      size_t begin = 0;
      double nearest = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < ring.size(); ++i)
        if (ring[i].Distance(near) < nearest) {
          nearest = ring[i].Distance(near);
          begin = i;
        }
      const size_t count = ring.size();
      F2CSwaths edges;
      for (size_t j = 0; j < count; ++j) {
        const auto a = ring[(begin + j) % count],
                   b = ring[(begin + j + 1) % count];
        const double length = a.Distance(b);
        if (length <= 1e-3) {
          continue;
        }
        // Keep every simplified edge.  Long edges get the full turning-radius
        // trim; short notches use a proportionally smaller trim so a narrow
        // doorway cannot erase the rest of the perimeter.
        const double trim =
            std::min(1.25 * robot.GetMinTurningRadius(), 0.25 * length);
        edges.emplace_back(
            F2CLineString(a.GetPointAlong(b, trim), b.GetPointAlong(a, trim)),
            config_.operation_width);
      }
      if (edges.size() == 0) {
        continue;
      }
      if (!hole) {
        AppendFallbackRing(path, edges, robot, *fallback_turn, config_);
        continue;
      }
      F2CRoute route;
      for (const auto& edge : edges)
        route.AddConnectedSwaths(F2CMultiPoint(), F2CSwaths({edge}));
      const auto& first = edges[0];
      // Closing is added through the same route connector rather than accepting
      // a raw arc.
      route.AddConnectedSwaths(
          F2CMultiPoint(),
          F2CSwaths({F2CSwath(
              F2CLineString(
                  first.StartPoint(),
                  first.StartPoint() + F2CPoint(std::cos(first.GetInAngle()),
                                                std::sin(first.GetInAngle())) *
                                           1e-5),
              config_.operation_width)}));
      F2CPath hole_path;
      try {
        route.SetField(field);
        hole_path = f2c::pp::PathPlanning::PlanPath(robot, route,
                                                    *fallback_turn);
      } catch (const std::exception&) {
        hole_path = F2CPath();
      }
      if (hole_path.size() > 1) {
        AppendConnector(path, hole_path.AtStart(), hole_path[0].angle, robot,
                        *fallback_turn, config_);
        path += hole_path;
      } else {
        AppendBoundaryRing(path, ring, config_);
      }
    }
  return path;
}

}  // namespace autonomy::task::coverage
