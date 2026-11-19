// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file robot_params.cpp
 * @brief Native complete-coverage robot params implementation. */
#include "autonomy/task/navigation/coverage/robot_params.hpp"

#include "autonomy/task/navigation/coverage/generator_utils.hpp"
namespace autonomy::task::coverage {
RobotParams::RobotParams(const Configuration& c)
    : robot_(c.robot_width, c.operation_width) {
  Positive(c.robot_width, "robot_width");
  Positive(c.operation_width, "operation_width");
  Positive(c.min_turning_radius, "min_turning_radius");
  Positive(c.linear_curv_change, "linear_curv_change");
  Positive(c.cruise_speed, "cruise_speed");
  Positive(c.turn_speed, "turn_speed");
  robot_.SetMinTurningRadius(c.min_turning_radius);
  robot_.SetMaxDiffCurv(c.linear_curv_change);
  robot_.SetCruiseVel(c.cruise_speed);
  robot_.SetTurnVel(c.turn_speed);
}

}  // namespace autonomy::task::coverage
