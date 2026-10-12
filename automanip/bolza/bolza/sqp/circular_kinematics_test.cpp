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

#include "bolza/sqp/sqp_solver.hpp"

#include <bolza/core/initialization/default_initializer.hpp>

#include "helpers/circular_kinematics.hpp"

TEST(test_circular_kinematics, solve_projected_EqConstraints) {
  // optimal control problem
  bolza::OptimalControlProblem problem = bolza::CreateCircularKinematicsProblem("/tmp/bolza/sqp_test_generated");

  // Initializer
  bolza::DefaultInitializer zeroInitializer(2);

  // Solver settings
  bolza::sqp::Settings settings;
  settings.dt = 0.01;
  settings.sqpIteration = 20;
  settings.projectStateInputEqualityConstraints = true;
  settings.useFeedbackPolicy = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  settings.nThreads = 1;

  // Additional problem definitions
  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 1.0;
  const bolza::vector_t initState = (bolza::vector_t(2) << 1.0, 0.0).finished();  // radius 1.0

  // Solve
  bolza::SqpSolver solver(settings, problem, zeroInitializer);
  solver.Run(startTime, initState, finalTime);

  // Inspect solution
  const auto primalSolution = solver.GetPrimalSolution(finalTime);
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << primalSolution.timeTrajectory_[i] << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.GetPerformanceIndeces();
  ASSERT_LT(performance.dynamicsViolationSSE, 1e-6);
  ASSERT_LT(performance.equalityConstraintsSSE, 1e-6);

  // Check feedback controller
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    // Feed forward part
    ASSERT_TRUE(u.isApprox(primalSolution.controllerPtr_->ComputeInput(t, x)));
  }
}

TEST(test_circular_kinematics, solve_EqConstraints_inQPSubproblem) {
  // optimal control problem
  bolza::OptimalControlProblem problem = bolza::CreateCircularKinematicsProblem("/tmp/sqp_test_generated");

  // Initializer
  bolza::DefaultInitializer zeroInitializer(2);

  // Solver settings
  bolza::sqp::Settings settings;
  settings.dt = 0.01;
  settings.sqpIteration = 20;
  settings.projectStateInputEqualityConstraints = false;  // <- false to turn off projection of state-input equalities
  settings.useFeedbackPolicy = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;

  // Additional problem definitions
  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 1.0;
  const bolza::vector_t initState = (bolza::vector_t(2) << 1.0, 0.0).finished();  // radius 1.0

  // Solve
  bolza::SqpSolver solver(settings, problem, zeroInitializer);
  solver.Run(startTime, initState, finalTime);

  // Inspect solution
  const auto primalSolution = solver.GetPrimalSolution(finalTime);
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << primalSolution.timeTrajectory_[i] << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.GetPerformanceIndeces();
  ASSERT_LT(performance.dynamicsViolationSSE, 1e-6);
  ASSERT_LT(performance.equalityConstraintsSSE, 1e-6);

  // Check feedback controller
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    // Feed forward part
    ASSERT_TRUE(u.isApprox(primalSolution.controllerPtr_->ComputeInput(t, x)));
  }
}
