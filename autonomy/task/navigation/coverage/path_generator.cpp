// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file path_generator.cpp
 * @brief Native complete-coverage path generator implementation. */
#include "autonomy/task/navigation/coverage/path_generator.hpp"

#include "autonomy/task/navigation/coverage/generator_utils.hpp"
#include "autonomy/task/navigation/coverage/robot_params.hpp"
namespace autonomy::task::coverage {
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
  auto turn = CreateCurve(m);
  F2CRoute route;
  for (const auto& cell : rings)
    for (const auto& ring : cell) {
      if (ring.size() < 4) continue;
      size_t begin = 0;
      double nearest = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i + 1 < ring.size(); ++i)
        if (ring[i].Distance(near) < nearest) {
          nearest = ring[i].Distance(near);
          begin = i;
        }
      const size_t count = ring.size() - 1;
      F2CSwaths edges;
      for (size_t j = 0; j < count; ++j) {
        const auto a = ring[(begin + j) % count],
                   b = ring[(begin + j + 1) % count];
        const double length = a.Distance(b);
        const double trim = 1.25 * robot.GetMinTurningRadius();
        if (length <= 2 * trim)
          throw RequestException(
              "Headland perimeter has an edge too short for the turning "
              "radius");
        edges.emplace_back(
            F2CLineString(a.GetPointAlong(b, trim), b.GetPointAlong(a, trim)),
            config_.operation_width);
      }
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
    }
  route.SetField(field);
  auto path = f2c::pp::PathPlanning::PlanPath(robot, route, *turn);
  return path;
}

}  // namespace autonomy::task::coverage
