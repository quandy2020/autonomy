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

#include <bolza/core/types.hpp>
#include <bolza/core/constraint/state_input_constraint.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>

constexpr size_t STATE_DIM = 3;
constexpr size_t INPUT_DIM = 1;

namespace bolza {

/* Hybrid system dynamics */

class HybridSysDynamics1 final : public LinearSystemDynamics {
 public:
  HybridSysDynamics1() : LinearSystemDynamics(matrix_t(STATE_DIM, STATE_DIM), matrix_t(STATE_DIM, INPUT_DIM)) {
    LinearSystemDynamics::A_ << -0.1, 0.9, 0.0, -1.0, -0.01, 0.0, 0.0, 0.0, 0.0;
    LinearSystemDynamics::B_ << 0.0, 1.0, 0.0;
  };
  ~HybridSysDynamics1() override = default;

  vector_t ComputeJumpMap(scalar_t time, const vector_t& state, const PreComputation&) override {
    vector_t mappedState = state;
    mappedState[2] = 1;
    return mappedState;
  }

  vector_t ComputeGuardSurfaces(scalar_t time, const vector_t& state) override {
    vector_t GuardSurfaces(2);
    guardSurfaces[0] = 1;
    guardSurfaces[1] = -state[0] * state[1];
    return guardSurfaces;
  }

  HybridSysDynamics1* Clone() const override { return new HybridSysDynamics1(*this); }
};

class HybridSysDynamics2 final : public LinearSystemDynamics {
 public:
  HybridSysDynamics2() : LinearSystemDynamics(matrix_t(STATE_DIM, STATE_DIM), matrix_t(STATE_DIM, INPUT_DIM)) {
    LinearSystemDynamics::A_ << -0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    LinearSystemDynamics::B_ << 0.0, 1.0, 0.0;
  };
  ~HybridSysDynamics2() override = default;

  vector_t ComputeJumpMap(scalar_t time, const vector_t& state, const PreComputation&) override {
    vector_t mappedState = state;
    mappedState[2] = 0;
    return mappedState;
  }

  vector_t ComputeGuardSurfaces(scalar_t time, const vector_t& state) override {
    vector_t GuardSurfaces(2);
    guardSurfaces[0] = state[0] * state[1];
    guardSurfaces[1] = 1;
    return guardSurfaces;
  }

  HybridSysDynamics2* Clone() const final { return new HybridSysDynamics2(*this); }
};

class HybridSysDynamics final : public SystemDynamicsBase {
 public:
  HybridSysDynamics() {
    subsystemDynamicsPtr_[0].Reset(new HybridSysDynamics1);
    subsystemDynamicsPtr_[1].Reset(new HybridSysDynamics2);
  }

  ~HybridSysDynamics() override = default;

  HybridSysDynamics* Clone() const override { return new HybridSysDynamics(*this); }

  HybridSysDynamics(const HybridSysDynamics& other) : subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].Reset(other.subsystemDynamicsPtr_[0]->Clone());
    subsystemDynamicsPtr_[1].Reset(other.subsystemDynamicsPtr_[1]->Clone());
  }

  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) override {
    size_t activeSubsystem = x[2];
    return subsystemDynamicsPtr_[activeSubsystem]->ComputeFlowMap(t, x, u, preComp);
  }

  vector_t ComputeJumpMap(scalar_t time, const vector_t& state, const PreComputation& preComp) override {
    size_t activeSubsystem = state[2];
    return subsystemDynamicsPtr_[activeSubsystem]->ComputeJumpMap(time, state, preComp);
  }

  vector_t ComputeGuardSurfaces(scalar_t time, const vector_t& state) override {
    size_t activeSubsystem = state[2];
    return subsystemDynamicsPtr_[activeSubsystem]->ComputeGuardSurfaces(time, state);
  }

  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override {
    size_t activeSubsystem = x[2];
    return subsystemDynamicsPtr_[activeSubsystem]->LinearApproximation(t, x, u, preComp);
  }

 private:
  std::vector<std::unique_ptr<SystemDynamicsBase>> subsystemDynamicsPtr_{2};
};

/* Hybrid system constraints */

class HybridSysBounds final : public StateInputConstraint {
 public:
  HybridSysBounds() : StateInputConstraint(ConstraintOrder::Quadratic) {}
  ~HybridSysBounds() override = default;

  HybridSysBounds* Clone() const override { return new HybridSysBounds(*this); }

  size_t GetNumConstraints(scalar_t time) const override { return 4; }

  vector_t GetValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override {
    vector_t h(4);
    // clang-format off
    h << -u[0] + 2, // -2 < u < 2
          u[0] + 2,
          x[0] + 2, // -2 < x < 2
         -x[0] + 2;
    // clang-format on
    return h;
  }

  VectorFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const PreComputation& preComp) const override {
    VectorFunctionQuadraticApproximation h;
    h.f = GetValue(t, x, u, preComp);
    h.dfdx.setZero(4, STATE_DIM);
    h.dfdx.row(2) << 1, 0, 0;
    h.dfdx.row(3) << -1, 0, 0;
    h.dfdu.setZero(4, INPUT_DIM);
    h.dfdu << -1, 1, 0, 0;
    h.dfdxx.resize(4, matrix_t::Zero(STATE_DIM, STATE_DIM));
    h.dfduu.resize(4, matrix_t::Zero(INPUT_DIM, INPUT_DIM));
    h.dfdux.resize(4, matrix_t::Zero(INPUT_DIM, STATE_DIM));
    return h;
  }
};

}  // namespace bolza
