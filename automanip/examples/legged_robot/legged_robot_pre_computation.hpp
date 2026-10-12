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

#pragma once

#include <memory>
#include <string>

#include <bolza/core/pre_computation.hpp>
#include <bolza/pinocchio/interface/pinocchio_interface.hpp>

#include <bolza/pinocchio/centroidal/centroidal_model_pinocchio_mapping.hpp>

#include "examples/legged_robot/common/model_settings.hpp"
#include "examples/legged_robot/constraint/end_effector_linear_constraint.hpp"
#include "examples/legged_robot/foot_planner/swing_trajectory_planner.hpp"

namespace bolza {
namespace legged_robot {

/** Callback for caching and reference update */
class LeggedRobotPreComputation : public PreComputation {
 public:
  LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                            const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings);
  ~LeggedRobotPreComputation() override = default;

  LeggedRobotPreComputation* Clone() const override;

  void Request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  const std::vector<EndEffectorLinearConstraint::Config>& GetEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

  PinocchioInterface& GetPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& GetPinocchioInterface() const { return pinocchioInterface_; }

 private:
  LeggedRobotPreComputation(const LeggedRobotPreComputation& other) = default;

  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  const ModelSettings settings_;

  std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
};

}  // namespace legged_robot
}  // namespace bolza
