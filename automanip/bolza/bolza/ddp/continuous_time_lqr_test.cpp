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

#include <gtest/gtest.h>

#include "bolza/ddp/continuous_time_lqr.hpp"

#include <bolza/core/cost/quadratic_state_input_cost.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>

#include "helpers/problems_generation_test.hpp"

using namespace bolza;

TEST(testContinousTimeLqr, compareWithMatlab) {
  // Set up problem with arbitrary values
  const matrix_t A = (matrix_t(2, 2) << 1.0, 2.0, 3.0, 4.0).finished();
  const matrix_t B = (matrix_t(2, 1) << 5.0, 6.0).finished();
  LinearSystemDynamics dynamics(A, B);

  const matrix_t Q = (matrix_t(2, 2) << 3.0, 2.0, 2.0, 4.0).finished();
  const matrix_t R = (matrix_t(1, 1) << 5.0).finished();
  const matrix_t P = (matrix_t(1, 2) << 0.1, 0.2).finished();
  auto cost = std::make_unique<QuadraticStateInputCost>(Q, R, P);

  const scalar_t time = 0.0;
  const vector_t state = vector_t::Zero(2);
  const vector_t input = vector_t::Zero(1);
  const TargetTrajectories TargetTrajectories({time}, {state}, {input});

  bolza::OptimalControlProblem problem;
  problem.dynamicsPtr.reset(dynamics.Clone());
  problem.costPtr->Add("cost", std::move(cost));
  problem.targetTrajectoriesPtr = &targetTrajectories;

  // Solve LQR
  const auto lqrSolution = continuous_time_lqr::Solve(problem, time, state, input);

  // MATLAB results, notice sign difference with MATLAB, bolza computes K for u = K * x
  const matrix_t K_check = (matrix_t(1, 2) << -0.905054653909129, -1.802904101100247).finished();
  const matrix_t S_check = (matrix_t(2, 2) << 1.109884545577592, -0.187358243057052, -0.187358243057052, 1.625218620131083).finished();

  const scalar_t tolerance = 1e-9;
  ASSERT_TRUE(lqrSolution.feedbackGains.isApprox(K_check, tolerance));
  ASSERT_TRUE(lqrSolution.valueFunction.isApprox(S_check, tolerance));
}

TEST(testContinousTimeLqr, evaluateCAREresidual) {
  const scalar_t careResidualNormTolerance = 1e-9;  // CARE needs to be solved up until this tolerance

  // Check random problems of increasing size.
  for (int n = 2; n < 65; n *= 2) {
    const int m = n / 2;

    // random dynamics
    const auto dynamicsMatrices = GetRandomDynamics(n, m);
    const auto dynamics = GetOcs2Dynamics(dynamicsMatrices);

    // random costs
    const auto costMatrices = GetRandomCost(n, m);
    auto cost = GetOcs2Cost(costMatrices);
    const scalar_t time = 0.0;
    const vector_t state = vector_t::Random(n);
    const vector_t input = vector_t::Random(m);
    const TargetTrajectories TargetTrajectories({time}, {state}, {input});

    bolza::OptimalControlProblem problem;
    problem.dynamicsPtr.reset(dynamics->Clone());
    problem.costPtr->Add("cost", std::move(cost));
    problem.targetTrajectoriesPtr = &targetTrajectories;

    // Solve LQR
    const scalar_t timeLinearization = 0.0;
    const vector_t stateLinearization = vector_t::Random(n);
    const vector_t inputLinearization = vector_t::Random(m);
    const auto lqrSolution = continuous_time_lqr::Solve(problem, timeLinearization, stateLinearization, inputLinearization);

    // Check CARE
    const auto& A = dynamicsMatrices.dfdx;
    const auto& B = dynamicsMatrices.dfdu;
    const auto& Q = costMatrices.dfdxx;
    const auto& R = costMatrices.dfduu;
    const auto& P = costMatrices.dfdux;
    const auto& S = lqrSolution.valueFunction;
    const matrix_t careResidual = A.transpose() * S + S * A - (S * B + P.transpose()) * R.lu().solve(B.transpose() * S + P) + Q;
    ASSERT_LT(careResidual.norm(), careResidualNormTolerance);
  }
}
