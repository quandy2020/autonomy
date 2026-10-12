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

#include <utility>

#include <bolza/core/types.hpp>
#include <bolza/core/cost/state_cost.hpp>
#include <bolza/core/cost/state_input_cost.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>

#include "helpers/bouncingmass/overall_reference.hpp"

static constexpr size_t STATE_DIM = 3;
static constexpr size_t INPUT_DIM = 1;

using scalar_t = bolza::scalar_t;
using vector_t = bolza::vector_t;
using matrix_t = bolza::matrix_t;
using PreComputation = bolza::PreComputation;
using ScalarFunctionQuadraticApproximation = bolza::ScalarFunctionQuadraticApproximation;

class BouncingMassDynamics final : public bolza::LinearSystemDynamics {
 public:
  BouncingMassDynamics()
      : LinearSystemDynamics(matrix_t(STATE_DIM, STATE_DIM), matrix_t(STATE_DIM, INPUT_DIM), matrix_t(STATE_DIM, STATE_DIM)) {
    constexpr scalar_t e = 0.95;
    A_ << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    B_ << 0.0, 1.0, 0.0;
    G_ << 1.0, 0.0, 0.0, 0.0, -e, 0.0, 0.0, 0.0, 1.0;
  }

  ~BouncingMassDynamics() override = default;

  BouncingMassDynamics* Clone() const override { return new BouncingMassDynamics(*this); }

  vector_t ComputeJumpMap(scalar_t t, const vector_t& x, const PreComputation& preComp) override {
    vector_t mappedState = LinearSystemDynamics::ComputeJumpMap(t, x, preComp);
    if (x(2) < 5) {
      MappedState(2) += 1;
    }
    return mappedState;
  }

  vector_t ComputeGuardSurfaces(scalar_t t, const vector_t& x) override {
    vector_t GuardSurfaces(1);
    GuardSurfaces(0) = x(0);
    return guardSurfaces;
  }

  bolza::VectorFunctionLinearApproximation GuardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) override {
    bolza::VectorFunctionLinearApproximation approximation;
    approximation.dfdx.setIdentity(1, x.rows());
    approximation.dfdu.setZero(1, u.rows());
    approximation.f = ComputeGuardSurfaces(t, x);
    return approximation;
  }
};

class BouncingMassCost final : public bolza::StateInputCost {
 public:
  BouncingMassCost(OverallReference ref, matrix_t Q, matrix_t R) : ref_(std::move(ref)), Q_(std::move(Q)), R_(std::move(R)) {}

  ~BouncingMassCost() override = default;

  BouncingMassCost* Clone() const override { return new BouncingMassCost(*this); }

  scalar_t GetValue(scalar_t t, const vector_t& x, const vector_t& u, const bolza::TargetTrajectories&,
                    const PreComputation&) const override {
    const auto stateInput = GetNominalStateInput(t, x, u);
    const vector_t xDeviation = x - stateInput.first;
    const vector_t uDeviation = u - stateInput.second;
    return 0.5 * xDeviation.dot(Q_ * xDeviation) + 0.5 * uDeviation.dot(R_ * uDeviation);
  }

  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const bolza::TargetTrajectories&, const PreComputation&) const override {
    const auto stateInput = GetNominalStateInput(t, x, u);
    const vector_t xDeviation = x - stateInput.first;
    const vector_t uDeviation = u - stateInput.second;

    ScalarFunctionQuadraticApproximation L;
    L.f = 0.5 * xDeviation.dot(Q_ * xDeviation) + 0.5 * uDeviation.dot(R_ * uDeviation);
    L.dfdx = Q_ * xDeviation;
    L.dfdu = R_ * uDeviation;
    L.dfdxx = Q_;
    L.dfdux.setZero(INPUT_DIM, STATE_DIM);
    L.dfduu = R_;
    return L;
  }

 private:
  std::pair<vector_t, vector_t> GetNominalStateInput(scalar_t t, const vector_t& x, const vector_t& u) const {
    const int currentMode = x.tail(1).Value();
    return {ref_.getState(currentMode, t), ref_.getInput(t)};
  }

  const OverallReference ref_;
  const matrix_t Q_;
  const matrix_t R_;
};

class BouncingMassFinalCost final : public bolza::StateCost {
 public:
  BouncingMassFinalCost(OverallReference ref, matrix_t QFinal, scalar_t timeFinal)
      : ref_(std::move(ref)), timeFinal_(timeFinal), QFinal_(std::move(QFinal)) {}

  ~BouncingMassFinalCost() override = default;

  BouncingMassFinalCost* Clone() const override { return new BouncingMassFinalCost(*this); }

  scalar_t GetValue(scalar_t t, const vector_t& x, const bolza::TargetTrajectories&, const PreComputation&) const override {
    const vector_t xDeviation = x - GetNominalFinalState(t, x);
    return 0.5 * xDeviation.dot(QFinal_ * xDeviation);
  }

  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const vector_t& x, const bolza::TargetTrajectories&,
                                                                 const PreComputation&) const override {
    const vector_t xDeviation = x - GetNominalFinalState(t, x);
    ScalarFunctionQuadraticApproximation Phi;
    Phi.f = 0.5 * xDeviation.dot(QFinal_ * xDeviation);
    Phi.dfdx = QFinal_ * xDeviation;
    Phi.dfdxx = QFinal_;
    return Phi;
  }

 private:
  vector_t GetNominalFinalState(scalar_t t, const vector_t& x) const {
    // Terminal cost only calculated for final state, not for intermediate switch states
    if (std::fabs(t - timeFinal_) > bolza::numeric_traits::WeakEpsilon<scalar_t>()) {
      return x;
    } else {
      const int currentMode = x.tail(1).Value();
      return ref_.getState(currentMode, t);
    }
  }

  const OverallReference ref_;
  const scalar_t timeFinal_;
  const matrix_t QFinal_;
};
