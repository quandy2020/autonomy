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

#include <bolza/oc/synchronized_module/reference_manager.hpp>
#include "helpers/problems_generation_test.hpp"

namespace bolza {
namespace {

std::pair<PrimalSolution, std::vector<PerformanceIndex>> solveWithFeedbackSetting(
    bool feedback, bool emptyConstraint, const VectorFunctionLinearApproximation& dynamicsMatrices,
    const ScalarFunctionQuadraticApproximation& costMatrices) {
  int n = dynamicsMatrices.dfdu.rows();
  int m = dynamicsMatrices.dfdu.cols();

  bolza::OptimalControlProblem problem;

  // System
  problem.dynamicsPtr = GetOcs2Dynamics(dynamicsMatrices);

  // Cost
  problem.costPtr->Add("intermediateCost", bolza::GetOcs2Cost(costMatrices));
  problem.finalCostPtr->Add("finalCost", bolza::GetOcs2StateCost(costMatrices));

  // Reference Managaer
  bolza::TargetTrajectories TargetTrajectories({0.0}, {bolza::vector_t::Ones(n)}, {bolza::vector_t::Ones(m)});
  auto referenceManagerPtr = std::make_shared<ReferenceManager>(targetTrajectories);

  problem.targetTrajectoriesPtr = &referenceManagerPtr->GetTargetTrajectories();

  if (emptyConstraint) {
    problem.equalityConstraintPtr->Add("intermediateCost", bolza::GetOcs2Constraints(GetRandomConstraints(n, m, 0)));
  }

  bolza::DefaultInitializer zeroInitializer(m);

  // Solver settings
  bolza::ipm::Settings settings;
  settings.dt = 0.05;
  settings.ipmIteration = 10;
  settings.useFeedbackPolicy = feedback;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  settings.nThreads = 100;

  // Additional problem definitions
  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 1.0;
  const bolza::vector_t initState = bolza::vector_t::Ones(n);

  // Construct solver
  bolza::IpmSolver solver(settings, problem, zeroInitializer);
  solver.SetReferenceManager(referenceManagerPtr);

  // Solve
  solver.Run(startTime, initState, finalTime);
  return {solver.GetPrimalSolution(finalTime), solver.GetIterationsLog()};
}

}  // namespace
}  // namespace bolza

TEST(test_unconstrained, withFeedback) {
  int n = 3;
  int m = 2;
  const double tol = 1e-9;
  const auto dynamics = bolza::GetRandomDynamics(n, m);
  const auto costs = bolza::GetRandomCost(n, m);
  const auto solWithEmptyConstraint = bolza::solveWithFeedbackSetting(true, true, dynamics, costs);
  const auto solWithNullConstraint = bolza::solveWithFeedbackSetting(true, false, dynamics, costs);

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(solWithEmptyConstraint.second.size(), 2);
  ASSERT_LE(solWithNullConstraint.second.size(), 2);
  ASSERT_LT(solWithEmptyConstraint.second.back().dynamicsViolationSSE, tol);
  ASSERT_LT(solWithNullConstraint.second.back().dynamicsViolationSSE, tol);

  // Compare
  const auto& withEmptyConstraint = solWithEmptyConstraint.first;
  const auto& withNullConstraint = solWithNullConstraint.first;
  for (int i = 0; i < withEmptyConstraint.timeTrajectory_.size(); i++) {
    ASSERT_DOUBLE_EQ(withEmptyConstraint.timeTrajectory_[i], withNullConstraint.timeTrajectory_[i]);
    ASSERT_TRUE(withEmptyConstraint.stateTrajectory_[i].isApprox(withNullConstraint.stateTrajectory_[i], tol));
    ASSERT_TRUE(withEmptyConstraint.inputTrajectory_[i].isApprox(withNullConstraint.inputTrajectory_[i], tol));
    const auto t = withEmptyConstraint.timeTrajectory_[i];
    const auto& x = withEmptyConstraint.stateTrajectory_[i];
    ASSERT_TRUE(
        withEmptyConstraint.controllerPtr_->ComputeInput(t, x).isApprox(withNullConstraint.controllerPtr_->ComputeInput(t, x), tol));
  }
}

TEST(test_unconstrained, noFeedback) {
  int n = 3;
  int m = 2;
  const double tol = 1e-9;
  const auto dynamics = bolza::GetRandomDynamics(n, m);
  const auto costs = bolza::GetRandomCost(n, m);
  const auto solWithEmptyConstraint = bolza::solveWithFeedbackSetting(false, true, dynamics, costs);
  const auto solWithNullConstraint = bolza::solveWithFeedbackSetting(false, false, dynamics, costs);

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(solWithEmptyConstraint.second.size(), 2);
  ASSERT_LE(solWithNullConstraint.second.size(), 2);
  ASSERT_LT(solWithEmptyConstraint.second.back().dynamicsViolationSSE, tol);
  ASSERT_LT(solWithNullConstraint.second.back().dynamicsViolationSSE, tol);

  // Compare
  const auto& withEmptyConstraint = solWithEmptyConstraint.first;
  const auto& withNullConstraint = solWithNullConstraint.first;
  for (int i = 0; i < withEmptyConstraint.timeTrajectory_.size(); i++) {
    ASSERT_DOUBLE_EQ(withEmptyConstraint.timeTrajectory_[i], withNullConstraint.timeTrajectory_[i]);
    ASSERT_TRUE(withEmptyConstraint.stateTrajectory_[i].isApprox(withNullConstraint.stateTrajectory_[i], tol));
    ASSERT_TRUE(withEmptyConstraint.inputTrajectory_[i].isApprox(withNullConstraint.inputTrajectory_[i], tol));

    const auto t = withEmptyConstraint.timeTrajectory_[i];
    const auto& x = withEmptyConstraint.stateTrajectory_[i];
    ASSERT_TRUE(
        withEmptyConstraint.controllerPtr_->ComputeInput(t, x).isApprox(withNullConstraint.controllerPtr_->ComputeInput(t, x), tol));
  }
}
