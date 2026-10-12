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

#include <bolza/core/initialization/default_initializer.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>
#include "helpers/problems_generation_test.hpp"

#include "bolza/slp/slp_solver.hpp"

namespace bolza {
namespace {

std::pair<PrimalSolution, std::vector<PerformanceIndex>> Solve(const VectorFunctionLinearApproximation& dynamicsMatrices,
                                                               const ScalarFunctionQuadraticApproximation& costMatrices,
                                                               const bolza::scalar_t tol) {
  int n = dynamicsMatrices.dfdu.rows();
  int m = dynamicsMatrices.dfdu.cols();

  bolza::OptimalControlProblem problem;

  // System
  problem.dynamicsPtr = GetOcs2Dynamics(dynamicsMatrices);

  // Cost
  problem.costPtr->Add("intermediateCost", bolza::GetOcs2Cost(costMatrices));
  problem.finalCostPtr->Add("finalCost", bolza::GetOcs2StateCost(costMatrices));

  // Reference Manager
  bolza::TargetTrajectories TargetTrajectories({0.0}, {bolza::vector_t::Ones(n)}, {bolza::vector_t::Ones(m)});
  std::shared_ptr<ReferenceManager> referenceManagerPtr(new ReferenceManager(targetTrajectories));

  problem.targetTrajectoriesPtr = &referenceManagerPtr->GetTargetTrajectories();

  problem.equalityConstraintPtr->Add("intermediateCost", bolza::GetOcs2Constraints(GetRandomConstraints(n, m, 0)));

  bolza::DefaultInitializer zeroInitializer(m);

  // Solver settings
  auto getPipgSettings = [&]() {
    bolza::pipg::Settings settings;
    settings.maxNumIterations = 30000;
    settings.absoluteTolerance = tol;
    settings.relativeTolerance = 1e-2;
    settings.lowerBoundH = 1e-3;
    settings.checkTerminationInterval = 1;
    settings.displayShortSummary = true;
    return settings;
  };

  const auto slpSettings = [&]() {
    bolza::slp::Settings settings;
    settings.dt = 0.05;
    settings.slpIteration = 10;
    settings.scalingIteration = 3;
    settings.printSolverStatistics = true;
    settings.printSolverStatus = true;
    settings.printLinesearch = true;
    settings.nThreads = 100;
    settings.pipgSettings = getPipgSettings();
    return settings;
  }();

  // Additional problem definitions
  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 1.0;
  const bolza::vector_t initState = bolza::vector_t::Ones(n);

  // Construct solver
  bolza::SlpSolver solver(slpSettings, problem, zeroInitializer);
  solver.SetReferenceManager(referenceManagerPtr);

  // Solve
  solver.Run(startTime, initState, finalTime);
  return {solver.GetPrimalSolution(finalTime), solver.GetIterationsLog()};
}

}  // namespace
}  // namespace bolza

TEST(testSlpSolver, test_unconstrained) {
  int n = 3;
  int m = 2;
  const double tol = 1e-9;
  const auto dynamics = bolza::GetRandomDynamics(n, m);
  const auto costs = bolza::GetRandomCost(n, m);
  const auto result = bolza::Solve(dynamics, costs, tol);

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(result.second.size(), 2);
  ASSERT_LT(result.second.back().dynamicsViolationSSE, tol);
}
