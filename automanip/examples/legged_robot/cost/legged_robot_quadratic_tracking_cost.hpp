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

#include <bolza/pinocchio/centroidal/centroidal_model_info.hpp>
#include <bolza/core/cost/quadratic_state_cost.hpp>
#include <bolza/core/cost/quadratic_state_input_cost.hpp>

#include "examples/legged_robot/common/utils.hpp"
#include "examples/legged_robot/reference_manager/switched_model_reference_manager.hpp"

namespace bolza {
namespace legged_robot {

/**
 * State-input tracking cost used for intermediate times
 */
class LeggedRobotStateInputQuadraticCost final : public QuadraticStateInputCost {
 public:
  LeggedRobotStateInputQuadraticCost(matrix_t Q, matrix_t R, CentroidalModelInfo info,
                                     const SwitchedModelReferenceManager& referenceManager)
      : QuadraticStateInputCost(std::move(Q), std::move(R)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

  ~LeggedRobotStateInputQuadraticCost() override = default;
  LeggedRobotStateInputQuadraticCost* Clone() const override { return new LeggedRobotStateInputQuadraticCost(*this); }

 private:
  LeggedRobotStateInputQuadraticCost(const LeggedRobotStateInputQuadraticCost& rhs) = default;

  std::pair<vector_t, vector_t> GetStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const vector_t xNominal = targetTrajectories.GetDesiredState(time);
    const vector_t uNominal = WeightCompensatingInput(info_, contactFlags);
    return {state - xNominal, input - uNominal};
  }

  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
};

/**
 * State tracking cost used for the final time
 */
class LeggedRobotStateQuadraticCost final : public QuadraticStateCost {
 public:
  LeggedRobotStateQuadraticCost(matrix_t Q, CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager)
      : QuadraticStateCost(std::move(Q)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

  ~LeggedRobotStateQuadraticCost() override = default;
  LeggedRobotStateQuadraticCost* Clone() const override { return new LeggedRobotStateQuadraticCost(*this); }

 private:
  LeggedRobotStateQuadraticCost(const LeggedRobotStateQuadraticCost& rhs) = default;

  vector_t GetStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const vector_t xNominal = targetTrajectories.GetDesiredState(time);
    return state - xNominal;
  }

  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
};

}  // namespace legged_robot
}  // namespace bolza
