// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file robot_params.hpp
 * @brief RobotParams stage adapter for native Fields2Cover coverage.
 */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief RobotParams single coverage-stage responsibility. */
class RobotParams {
 public:
  /** @brief Validate and hold the robot kinematic and operational parameters.
   */
  explicit RobotParams(const Configuration& config);
  /** @brief Return the configured robot model. */
  F2CRobot& GetRobot() { return robot_; }
  /** @brief Return the operational swath width in metres. */
  double GetOperationWidth() const { return robot_.GetCovWidth(); }

 private:
  F2CRobot robot_;
};
}  // namespace autonomy::task::coverage
