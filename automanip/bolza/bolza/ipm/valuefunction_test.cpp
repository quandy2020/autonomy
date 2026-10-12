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

#include "bolza/ipm/ipm_solver.hpp"

#include <bolza/core/initialization/default_initializer.hpp>

#include <bolza/oc/oc_data/time_discretization.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>
#include "helpers/problems_generation_test.hpp"

TEST(test_valuefunction, linear_quadratic_problem) {
  constexpr int n = 3;
  constexpr int m = 2;
  constexpr int nc = 1;
  constexpr int Nsample = 10;
  constexpr bolza::scalar_t tol = 1e-9;
  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t eventTime = 1.0 / 3.0;
  const bolza::scalar_t finalTime = 1.0;

  bolza::OptimalControlProblem problem;

  // System
  const auto dynamics = bolza::GetRandomDynamics(n, m);
  const auto jumpMap = bolza::matrix_t::Random(n, n);
  problem.dynamicsPtr.reset(new bolza::LinearSystemDynamics(dynamics.dfdx, dynamics.dfdu, jumpMap));

  // Cost
  problem.costPtr->Add("intermediateCost", bolza::GetOcs2Cost(bolza::GetRandomCost(n, m)));
  problem.preJumpCostPtr->Add("eventCost", bolza::GetOcs2StateCost(bolza::GetRandomCost(n, 0)));
  problem.finalCostPtr->Add("finalCost", bolza::GetOcs2StateCost(bolza::GetRandomCost(n, 0)));

  // Reference Manager
  const bolza::ModeSchedule ModeSchedule({eventTime}, {0, 1});
  const bolza::TargetTrajectories TargetTrajectories({0.0}, {bolza::vector_t::Random(n)}, {bolza::vector_t::Random(m)});
  auto referenceManagerPtr = std::make_shared<bolza::ReferenceManager>(targetTrajectories, modeSchedule);

  problem.targetTrajectoriesPtr = &targetTrajectories;

  // Constraint
  problem.equalityConstraintPtr->Add("constraint", bolza::GetOcs2Constraints(bolza::GetRandomConstraints(n, m, nc)));

  bolza::DefaultInitializer zeroInitializer(m);

  // Solver settings
  bolza::ipm::Settings settings;
  settings.dt = 0.05;
  settings.ipmIteration = 1;
  settings.printSolverStatistics = false;
  settings.printSolverStatus = false;
  settings.printLinesearch = false;
  settings.useFeedbackPolicy = true;
  settings.createValueFunction = true;

  // Set up solver
  bolza::IpmSolver solver(settings, problem, zeroInitializer);
  solver.SetReferenceManager(referenceManagerPtr);

  // Get value function
  const bolza::vector_t zeroState = bolza::vector_t::Random(n);
  solver.Reset();
  solver.Run(startTime, zeroState, finalTime);
  const auto costToGo = solver.GetValueFunction(startTime, zeroState);
  const bolza::scalar_t zeroCost = solver.GetPerformanceIndeces().cost;

  // Solve for random states and check consistency with value function
  for (int i = 0; i < Nsample; ++i) {
    const bolza::vector_t sampleState = bolza::vector_t::Random(n);
    solver.Reset();
    solver.Run(startTime, sampleState, finalTime);
    const bolza::scalar_t sampleCost = solver.GetPerformanceIndeces().cost;
    const bolza::vector_t dx = sampleState - zeroState;

    EXPECT_NEAR(sampleCost, zeroCost + costToGo.dfdx.dot(dx) + 0.5 * dx.dot(costToGo.dfdxx * dx), tol);
  }
}
