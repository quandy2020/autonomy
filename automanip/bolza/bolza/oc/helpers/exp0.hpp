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

#include <bolza/core/cost/quadratic_state_cost.hpp>
#include <bolza/core/cost/quadratic_state_input_cost.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Sys1 final : public LinearSystemDynamics {
 public:
  EXP0_Sys1() : LinearSystemDynamics(matrix_t(2, 2), matrix_t(2, 1)) {
    LinearSystemDynamics::A_ << 0.6, 1.2, -0.8, 3.4;
    LinearSystemDynamics::B_ << 1, 1;
  }
  ~EXP0_Sys1() override = default;

  EXP0_Sys1* Clone() const override { return new EXP0_Sys1(*this); }

 private:
  EXP0_Sys1(const EXP0_Sys1& other) = default;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Sys2 final : public LinearSystemDynamics {
 public:
  EXP0_Sys2() : LinearSystemDynamics(matrix_t(2, 2), matrix_t(2, 1)) {
    LinearSystemDynamics::A_ << 4, 3, -1, 0;
    LinearSystemDynamics::B_ << 2, -1;
  };
  ~EXP0_Sys2() override = default;

  EXP0_Sys2* Clone() const override { return new EXP0_Sys2(*this); }

 private:
  EXP0_Sys2(const EXP0_Sys2& other) = default;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_System : public SystemDynamicsBase {
 public:
  explicit EXP0_System(std::shared_ptr<ReferenceManager> referenceManagerPtr)
      : referenceManagerPtr_(std::move(referenceManagerPtr)), subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].Reset(new EXP0_Sys1);
    subsystemDynamicsPtr_[1].Reset(new EXP0_Sys2);
  }

  ~EXP0_System() override = default;

  EXP0_System* Clone() const final { return new EXP0_System(*this); }

  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) final {
    const auto activeMode = referenceManagerPtr_->GetModeSchedule().ModeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->ComputeFlowMap(t, x, u, preComp);
  }

  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) final {
    const auto activeMode = referenceManagerPtr_->GetModeSchedule().ModeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->LinearApproximation(t, x, u, preComp);
  }

 private:
  EXP0_System(const EXP0_System& other) : EXP0_System(other.referenceManagerPtr_) {}

  std::vector<std::shared_ptr<SystemDynamicsBase>> subsystemDynamicsPtr_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Cost final : public QuadraticStateInputCost {
 public:
  EXP0_Cost() : QuadraticStateInputCost((matrix_t(2, 2) << 0.0, 0.0, 0.0, 1.0).finished(), matrix_t::Identity(1, 1)) {}
  ~EXP0_Cost() override = default;
  EXP0_Cost* Clone() const override { return new EXP0_Cost(*this); }

 private:
  EXP0_Cost(const EXP0_Cost& other) = default;

  std::pair<vector_t, vector_t> GetStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories) const override {
    return {state - targetTrajectories.stateTrajectory[0], input - targetTrajectories.inputTrajectory[0]};
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_FinalCost final : public QuadraticStateCost {
 public:
  EXP0_FinalCost() : QuadraticStateCost(matrix_t::Identity(2, 2)) {}
  ~EXP0_FinalCost() override = default;
  EXP0_FinalCost* Clone() const override { return new EXP0_FinalCost(*this); }

 private:
  EXP0_FinalCost(const EXP0_FinalCost& other) = default;

  vector_t GetStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    return state - targetTrajectories.stateTrajectory[0];
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::shared_ptr<ReferenceManager> GetExp0ReferenceManager(const scalar_array_t& eventTimes,
                                                                 const std::vector<size_t>& modeSequence) {
  const vector_t x = (vector_t(2) << 4.0, 2.0).finished();
  const vector_t u = (vector_t(1) << 0.0).finished();
  TargetTrajectories TargetTrajectories({0.0}, {x}, {u});

  ModeSchedule ModeSchedule(eventTimes, modeSequence);

  return std::make_shared<ReferenceManager>(std::move(targetTrajectories), std::move(modeSchedule));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline OptimalControlProblem CreateExp0Problem(const std::shared_ptr<bolza::ReferenceManager>& referenceManagerPtr) {
  // optimal control problem
  OptimalControlProblem problem;
  problem.dynamicsPtr.Reset(new EXP0_System(referenceManagerPtr));

  // cost function
  problem.costPtr->Add("cost", std::make_unique<bolza::EXP0_Cost>());
  problem.finalCostPtr->Add("finalCost", std::make_unique<bolza::EXP0_FinalCost>());

  return problem;
}

}  // namespace bolza
