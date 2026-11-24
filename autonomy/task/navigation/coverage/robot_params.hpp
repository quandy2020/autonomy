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
 * @file robot_params.hpp
 * @brief RobotParams stage adapter for native Fields2Cover coverage.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_ROBOT_PARAMS_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_ROBOT_PARAMS_HPP_

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/**
 * @class autonomy::task::coverage::RobotParams
 * @brief Validates the robot limits once and adapts them to Fields2Cover.
 *
 * Every stage that needs the robot model (headland, swath, path) builds it
 * through this adapter, so an invalid configuration fails at construction
 * instead of mid-plan.
 */
class RobotParams {
 public:
  /**
   * @brief Validates and stores the kinematic and operational parameters.
   * @param[in] config Coverage configuration with radius, width and turning
   *            radius already populated.
   * @throws RequestException when a limit is non-positive or inconsistent.
   */
  explicit RobotParams(const Configuration& config);

  /**
   * @brief Accesses the Fields2Cover robot model.
   * @return Mutable reference so a caller can adjust the turn velocity.
   */
  F2CRobot& GetRobot() { return robot_; }

  /**
   * @brief Reports the operational swath width.
   * @return Width in metres, as configured for the robot.
   */
  double GetOperationWidth() const { return robot_.GetCovWidth(); }

 private:
  // Fields2Cover robot model built from the configuration.
  F2CRobot robot_;
};
}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_ROBOT_PARAMS_HPP_
