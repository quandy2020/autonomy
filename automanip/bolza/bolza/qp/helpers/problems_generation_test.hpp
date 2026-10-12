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

#include "bolza/qp/qp_solver_types.hpp"
#include "bolza/qp/qp_trajectories.hpp"

// Shared OC generators (relative path avoids resolving to this file via -I bolza/qp).
#include "../../oc/helpers/problems_generation_test.hpp"

namespace bolza {
namespace qp_solver {

inline ContinuousTrajectory GetRandomTrajectory(int N, int n, int m, scalar_t dt) {
  ContinuousTrajectory trajectory;
  trajectory.timeTrajectory = scalar_array_t(N + 1);
  trajectory.stateTrajectory = vector_array_t(N + 1);
  trajectory.inputTrajectory = vector_array_t(N);
  auto t = -dt;
  std::generate(trajectory.timeTrajectory.begin(), trajectory.timeTrajectory.end(), [&t, dt]() {
    t += dt;
    return t;
  });
  std::generate(trajectory.stateTrajectory.begin(), trajectory.stateTrajectory.end(), [n]() { return vector_t::Random(n); });
  std::generate(trajectory.inputTrajectory.begin(), trajectory.inputTrajectory.end(), [m]() { return vector_t::Random(m); });
  return trajectory;
}

inline std::vector<LinearQuadraticStage> GenerateRandomLqProblem(int N, int nx, int nu, int nc) {
  std::vector<LinearQuadraticStage> lqProblem;
  lqProblem.reserve(N + 1);

  for (int k = 0; k < N; ++k) {
    lqProblem.emplace_back(GetRandomCost(nx, nu), GetRandomDynamics(nx, nu), GetRandomConstraints(nx, nu, nc));
  }

  // Terminal Cost
  lqProblem.emplace_back(GetRandomCost(nx, 0), VectorFunctionLinearApproximation(), GetRandomConstraints(nx, nu, nc));

  return lqProblem;
}

/** Checks QP feasibility and numerical conditioning */
inline bool IsQpFeasible(const bolza::ScalarFunctionQuadraticApproximation& qpCost,
                         const bolza::VectorFunctionLinearApproximation& qpConstraints) {
  const auto& H = qpCost.dfdxx;
  const auto& A = qpConstraints.dfdx;

  // Cost must be convex
  Eigen::LDLT<bolza::matrix_t> ldlt(H);
  if (!(H.ldlt().vectorD().array() > 0.0).all()) {
    std::cerr << "H is not positive definite\n";
    return false;
  }

  // Constraints feasibility
  Eigen::JacobiSVD<bolza::matrix_t> Svd(A);
  const auto conditionNumber = svd.singularValues()(0) / svd.singularValues().tail(1)(0);
  if (svd.Rank() != A.rows()) {
    std::cerr << "A is not full row-rank\n";
    return false;
  } else if (conditionNumber > 1e6) {
    std::cerr << "A is ill-conditioned\n";
    return false;
  }

  return true;
}

}  // namespace qp_solver
}  // namespace bolza
