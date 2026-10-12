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

#include <bolza/core/constraint/state_input_constraint.hpp>
#include <bolza/core/cost/state_input_cost_cpp_ad.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically modeled particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsSystem final : public SystemDynamicsBase {
 public:
  CircularKinematicsSystem() = default;
  ~CircularKinematicsSystem() override = default;

  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override { return u; }

  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) {
    VectorFunctionLinearApproximation dynamics;
    dynamics.f = ComputeFlowMap(t, x, u, preComp);
    dynamics.dfdx.setZero(2, 2);
    dynamics.dfdu.setIdentity(2, 2);
    return dynamics;
  }

  CircularKinematicsSystem* Clone() const override { return new CircularKinematicsSystem(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically modeled particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsCost : public StateInputCostCppAd {
 public:
  CircularKinematicsCost(const std::string& libraryPath = "/tmp/bolza") {
    Initialize(2, 2, 0, "circular_kinematics_cost", libraryPath, true, false);
  }

  CircularKinematicsCost* Clone() const override { return new CircularKinematicsCost(*this); }

  ad_scalar_t CostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                           const ad_vector_t& parameters) const override {
    return 0.5 * pow(state(0) * Input(1) - state(1) * Input(0) - 1.0, 2) + 0.005 * input.dot(input);
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This example defines an optimal control problem where a kinematically modeled particle is
 * supposed to orbit a unite circle (defined as a constraint) with velocity of 1[m/s]
 * (defined as a cost).
 */
class CircularKinematicsConstraints final : public StateInputConstraint {
 public:
  CircularKinematicsConstraints() : StateInputConstraint(ConstraintOrder::Linear) {}
  ~CircularKinematicsConstraints() override = default;

  CircularKinematicsConstraints* Clone() const override { return new CircularKinematicsConstraints(*this); }

  size_t GetNumConstraints(scalar_t time) const override { return 1; }

  vector_t GetValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override {
    vector_t e(1);
    e << x.dot(u);
    return e;
  }

  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation e;
    e.f = GetValue(t, x, u, preComp);
    e.dfdx = u.transpose();
    e.dfdu = x.transpose();
    return e;
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline OptimalControlProblem CreateCircularKinematicsProblem(const std::string& libraryFolder) {
  // optimal control problem
  OptimalControlProblem problem;
  problem.dynamicsPtr.reset(new CircularKinematicsSystem());

  // cost function
  problem.costPtr->Add("cost", std::make_unique<CircularKinematicsCost>(libraryFolder));

  // constraint
  problem.equalityConstraintPtr->Add("constraint", std::make_unique<CircularKinematicsConstraints>());

  return problem;
}

}  // namespace bolza
