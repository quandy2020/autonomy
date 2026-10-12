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

//
// Created by rgrandia on 26.02.20.
//

#pragma once

#include <bolza/core/constraint/linear_state_constraint.hpp>
#include <bolza/core/constraint/linear_state_input_constraint.hpp>
#include <bolza/core/cost/quadratic_state_cost.hpp>
#include <bolza/core/cost/quadratic_state_input_cost.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>

namespace bolza {

/** Get random positive definite costs of n states and m inputs */
inline ScalarFunctionQuadraticApproximation GetRandomCost(int n, int m) {
  matrix_t QPPR = matrix_t::Random(n + m, n + m);
  QPPR = QPPR.transpose() * QPPR;
  ScalarFunctionQuadraticApproximation cost;
  cost.dfdxx = QPPR.topLeftCorner(n, n);
  cost.dfdx = vector_t::Random(n);
  if (m >= 0) {
    cost.dfdux = QPPR.bottomLeftCorner(m, n);
    cost.dfduu = QPPR.bottomRightCorner(m, m);
    cost.dfdu = vector_t::Random(m);
  }
  cost.f = std::rand() / static_cast<scalar_t>(RAND_MAX);
  return cost;
}

inline std::unique_ptr<bolza::StateInputCost> GetOcs2Cost(const ScalarFunctionQuadraticApproximation& cost) {
  return std::make_unique<bolza::QuadraticStateInputCost>(cost.dfdxx, cost.dfduu, cost.dfdux);
}

inline std::unique_ptr<bolza::StateCost> GetOcs2StateCost(const ScalarFunctionQuadraticApproximation& costFinal) {
  return std::make_unique<bolza::QuadraticStateCost>(costFinal.dfdxx);
}

/** Get random linear dynamics of n states and m inputs */
inline VectorFunctionLinearApproximation GetRandomDynamics(int n, int m) {
  VectorFunctionLinearApproximation dynamics;
  dynamics.dfdx = matrix_t::Random(n, n);
  if (m >= 0) {
    dynamics.dfdu = matrix_t::Random(n, m);
  }
  dynamics.f = vector_t::Random(n);
  return dynamics;
}

inline std::unique_ptr<bolza::LinearSystemDynamics> GetOcs2Dynamics(const VectorFunctionLinearApproximation& dynamics) {
  return std::make_unique<bolza::LinearSystemDynamics>(dynamics.dfdx, dynamics.dfdu);
}

/** Get random nc linear constraints of n states, and m inputs */
inline VectorFunctionLinearApproximation GetRandomConstraints(int n, int m, int nc) {
  VectorFunctionLinearApproximation constraints;
  constraints.dfdx = matrix_t::Random(nc, n);
  if (m >= 0) {
    constraints.dfdu = matrix_t::Random(nc, m);
  }
  constraints.f = vector_t::Random(nc);
  return constraints;
}

inline std::unique_ptr<bolza::StateInputConstraint> GetOcs2Constraints(const VectorFunctionLinearApproximation& stateInputConstraints) {
  return std::make_unique<bolza::LinearStateInputConstraint>(stateInputConstraints.f, stateInputConstraints.dfdx,
                                                            stateInputConstraints.dfdu);
}

inline std::unique_ptr<bolza::StateConstraint> GetOcs2StateOnlyConstraints(const VectorFunctionLinearApproximation& stateOnlyConstraints) {
  return std::make_unique<bolza::LinearStateConstraint>(stateOnlyConstraints.f, stateOnlyConstraints.dfdx);
}

}  // namespace bolza
