/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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
 * @file solve_double_integrator.cpp
 * @brief bolza::Problem / bolza::Solve tutorial (1D double integrator, no Autolink).
 *
 *   x = [p; ṗ],  ẋ = A x + B u  (p̈ = u),  track x★ = [1; 0]
 *
 *   cmake --build <umbrella-build> --target bolza_tutorial_di && bolza_tutorial_di
 */

#include <iostream>

#include <bolza/core/types.hpp>
#include <bolza/solver.hpp>

int main() {
  using bolza::matrix_t;
  using bolza::vector_t;

  bolza::Problem problem;

  const matrix_t A = (matrix_t(2, 2) << 0, 1, 0, 0).finished();
  const matrix_t B = (matrix_t(2, 1) << 0, 1).finished();
  problem.SetLinearDynamics(A, B);

  const matrix_t Q = matrix_t::Identity(2, 2);
  const matrix_t R = matrix_t::Constant(1, 1, 0.1);
  problem.AddQuadraticCost(Q, R);
  problem.AddQuadraticFinalCost(10.0 * Q);

  problem.SetTarget((vector_t(2) << 1.0, 0.0).finished());
  problem.SetInitialState(vector_t::Zero(2));

  bolza::Solver::Options options;
  options.tf = 3.0;
  options.max_num_iterations = 20;
  options.display = true;

  bolza::Solver::Summary summary;
  bolza::Solve(options, &problem, &summary);

  std::cout << "open-loop  ok=" << summary.ok << "  " << summary.message
            << "  u(t0)=" << summary.u0.transpose()
            << "  x(tf)=" << summary.x_final.transpose() << '\n';

  options.mpc_steps = 50;
  options.time_horizon = 2.0;
  options.mrt_frequency_hz = 100.0;
  bolza::Solve(options, &problem, &summary);

  std::cout << "mpc        ok=" << summary.ok << "  " << summary.message
            << "  x=" << summary.x_final.transpose()
            << "  u=" << summary.u0.transpose() << '\n';
  return summary.ok ? 0 : 1;
}
