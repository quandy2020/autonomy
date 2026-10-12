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
class EXP1_Sys1 final : public SystemDynamicsBase {
 public:
  EXP1_Sys1() = default;
  ~EXP1_Sys1() override = default;

  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    vector_t Dxdt(2);
    Dxdt(0) = x(0) + u(0) * sin(x(0));
    Dxdt(1) = -x(1) - u(0) * cos(x(1));
    return dxdt;
  }

  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override {
    VectorFunctionLinearApproximation dynamics;
    dynamics.f = ComputeFlowMap(t, x, u, preComp);
    dynamics.dfdx.resize(2, 2);
    dynamics.dfdx << u(0) * cos(x(0)) + 1, 0, 0, u(0) * sin(x(1)) - 1;
    dynamics.dfdu.resize(2, 1);
    dynamics.dfdu << sin(x(0)), -cos(x(1));
    return dynamics;
  }

  EXP1_Sys1* Clone() const override { return new EXP1_Sys1(*this); }

 private:
  EXP1_Sys1(const EXP1_Sys1& other) = default;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys2 final : public SystemDynamicsBase {
 public:
  EXP1_Sys2() = default;
  ~EXP1_Sys2() override = default;

  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    vector_t Dxdt(2);
    Dxdt(0) = x(1) + u(0) * sin(x(1));
    Dxdt(1) = -x(0) - u(0) * cos(x(0));
    return dxdt;
  }

  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override {
    VectorFunctionLinearApproximation dynamics;
    dynamics.f = ComputeFlowMap(t, x, u, preComp);
    dynamics.dfdx.resize(2, 2);
    dynamics.dfdx << 0, u(0) * cos(x(1)) + 1, u(0) * sin(x(0)) - 1, 0;
    dynamics.dfdu.resize(2, 1);
    dynamics.dfdu << sin(x(1)), -cos(x(0));
    return dynamics;
  }

  EXP1_Sys2* Clone() const override { return new EXP1_Sys2(*this); }

 private:
  EXP1_Sys2(const EXP1_Sys2& other) = default;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys3 final : public SystemDynamicsBase {
 public:
  EXP1_Sys3() = default;
  ~EXP1_Sys3() override = default;

  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    vector_t Dxdt(2);
    Dxdt(0) = -x(0) - u(0) * sin(x(0));
    Dxdt(1) = x(1) + u(0) * cos(x(1));
    return dxdt;
  }

  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override {
    VectorFunctionLinearApproximation dynamics;
    dynamics.f = ComputeFlowMap(t, x, u, preComp);
    dynamics.dfdx.resize(2, 2);
    dynamics.dfdx << -u(0) * cos(x(0)) - 1, 0, 0, 1 - u(0) * sin(x(1));
    dynamics.dfdu.resize(2, 1);
    dynamics.dfdu << -sin(x(0)), cos(x(1));
    return dynamics;
  }

  EXP1_Sys3* Clone() const override { return new EXP1_Sys3(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_System final : public SystemDynamicsBase {
 public:
  EXP1_System(std::shared_ptr<ReferenceManager> referenceManagerPtr) : referenceManagerPtr_(std::move(referenceManagerPtr)) {
    subsystemDynamicsPtr_[0].reset(new EXP1_Sys1());
    subsystemDynamicsPtr_[1].reset(new EXP1_Sys2());
    subsystemDynamicsPtr_[2].reset(new EXP1_Sys3());
  }

  ~EXP1_System() override = default;

  EXP1_System* Clone() const override { return new EXP1_System(*this); }

  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) override {
    const auto activeMode = referenceManagerPtr_->GetModeSchedule().ModeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->ComputeFlowMap(t, x, u, preComp);
  }

  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override {
    const auto activeMode = referenceManagerPtr_->GetModeSchedule().ModeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->LinearApproximation(t, x, u, preComp);
  }

 private:
  EXP1_System(const EXP1_System& other) : EXP1_System(other.referenceManagerPtr_) {}

  std::shared_ptr<ReferenceManager> referenceManagerPtr_;
  std::vector<std::shared_ptr<SystemDynamicsBase>> subsystemDynamicsPtr_{3};
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Cost final : public QuadraticStateInputCost {
 public:
  EXP1_Cost() : QuadraticStateInputCost(matrix_t::Identity(2, 2), matrix_t::Identity(1, 1)) {}
  ~EXP1_Cost() override = default;
  EXP1_Cost* Clone() const override { return new EXP1_Cost(*this); }

 private:
  EXP1_Cost(const EXP1_Cost& other) = default;

  std::pair<vector_t, vector_t> GetStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories) const override {
    return {state - targetTrajectories.stateTrajectory[0], input - targetTrajectories.inputTrajectory[0]};
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_FinalCost final : public QuadraticStateCost {
 public:
  EXP1_FinalCost() : QuadraticStateCost(matrix_t::Identity(2, 2)) {}
  ~EXP1_FinalCost() override = default;
  EXP1_FinalCost* Clone() const override { return new EXP1_FinalCost(*this); }

 private:
  EXP1_FinalCost(const EXP1_FinalCost& other) = default;

  vector_t GetStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    return state - targetTrajectories.stateTrajectory[0];
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::shared_ptr<ReferenceManager> GetExp1ReferenceManager(const scalar_array_t& eventTimes,
                                                                 const std::vector<size_t>& modeSequence) {
  const vector_t x = (vector_t(2) << 1.0, -1.0).finished();
  const vector_t u = (vector_t(1) << 0.0).finished();
  TargetTrajectories TargetTrajectories({0.0}, {x}, {u});

  ModeSchedule ModeSchedule(eventTimes, modeSequence);

  return std::make_shared<ReferenceManager>(std::move(targetTrajectories), std::move(modeSchedule));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline OptimalControlProblem CreateExp1Problem(const std::shared_ptr<bolza::ReferenceManager>& referenceManagerPtr) {
  // optimal control problem
  OptimalControlProblem problem;
  problem.dynamicsPtr.reset(new bolza::EXP1_System(referenceManagerPtr));

  // cost function
  problem.costPtr->Add("cost", std::make_unique<bolza::EXP1_Cost>());
  problem.finalCostPtr->Add("finalCost", std::make_unique<bolza::EXP1_FinalCost>());

  return problem;
}

}  // namespace bolza
