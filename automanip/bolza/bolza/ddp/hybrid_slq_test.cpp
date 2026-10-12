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
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <bolza/oc/rollout/state_triggered_rollout.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>
#include <bolza/oc/synchronized_module/solver_observer.hpp>
#include "helpers/dynamics_hybrid_slq_test.hpp"

#include <bolza/core/augmented_lagrangian/augmented_lagrangian.hpp>
#include <bolza/core/cost/quadratic_state_cost.hpp>
#include <bolza/core/cost/quadratic_state_input_cost.hpp>
#include <bolza/core/initialization/operating_points.hpp>
#include <bolza/core/penalties/penalties.hpp>
#include <bolza/core/soft_constraint/state_input_soft_constraint.hpp>

#include <bolza/ddp/slq.hpp>

/*
 * Test for StateTriggeredRollout in combination with SLQ
 *
 * The system being tested is a combination of two Linear system
 * it switches between these systems on an event
 *
 * Guard Surfaces are:   x[0]*x[1] < 0 when in mode 0
 *                       x[0]*x[1] > 0 when in mode 1
 *
 * Cost function is:     x(t)^T Q x(t) + u(t)^T R u(t) + x(t1)^T P x(t1)^T
 *                       Q = P = [50,0;0,50];
 *                       R = 1;
 *
 * Constraints are:      |u|< 2
 *                       |x[0]| < 2
 *
 * The following tests are implemented and performed:
 * (1) No penetration of Guard Surfaces
 * (2) Constraint compliance
 * (3) Check of cost function compared against cost calculated during trusted run of SLQ
 */
TEST(HybridSlqTest, state_rollout_slq) {
  using namespace bolza;

  constexpr scalar_t minRelCost = 1e-6;  // to avoid early termination

  const ddp::Settings ddpSettings = [&]() {
    ddp::Settings settings;

    settings.algorithm_ = ddp::Algorithm::SLQ;
    settings.displayInfo_ = true;
    settings.displayShortSummary_ = true;
    settings.maxNumIterations_ = 100;
    settings.nThreads_ = 1;
    settings.minRelCost_ = minRelCost;
    settings.checkNumericalStability_ = false;
    settings.absTolODE_ = 1e-10;
    settings.relTolODE_ = 1e-7;
    settings.maxNumStepsPerSecond_ = 10000;
    settings.useFeedbackPolicy_ = true;
    settings.debugPrintRollout_ = false;
    settings.strategy_ = search_strategy::Type::LINE_SEARCH;
    settings.lineSearch_.minStepLength = 1e-3;

    return settings;
  }();

  const rollout::Settings rolloutSettings = [&]() {
    rollout::Settings settings;
    settings.absTolODE = 1e-10;
    settings.relTolODE = 1e-7;
    settings.timeStep = 1e-3;
    settings.maxNumStepsPerSecond = 10000;
    return settings;
  }();

  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 5.0;
  const vector_t initState = (vector_t(STATE_DIM) << 0.0, 1.0, 1.0).finished();

  // rollout
  HybridSysDynamics systemDynamics;
  StateTriggeredRollout stateTriggeredRollout(systemDynamics, rolloutSettings);

  // cost function
  const matrix_t Q = (matrix_t(STATE_DIM, STATE_DIM) << 50, 0, 0, 0, 50, 0, 0, 0, 0).finished();
  const matrix_t R = (matrix_t(INPUT_DIM, INPUT_DIM) << 1).finished();
  // constraints
  auto boundsConstraints = std::make_unique<HybridSysBounds>();

  OptimalControlProblem problem;
  problem.dynamicsPtr.Reset(systemDynamics.Clone());
  problem.costPtr->Add("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
  problem.preJumpCostPtr->Add("preJumpCost", std::make_unique<QuadraticStateCost>(Q));
  problem.finalCostPtr->Add("finalCost", std::make_unique<QuadraticStateCost>(Q));
  problem.inequalityLagrangianPtr->Add("bounds",
                                       Create(std::move(boundsConstraints), augmented::SlacknessSquaredHingePenalty::Create({200.0, 0.1})));

  const vector_t xNominal = vector_t::Zero(STATE_DIM);
  const vector_t uNominal = vector_t::Zero(INPUT_DIM);
  TargetTrajectories TargetTrajectories({startTime}, {xNominal}, {uNominal});
  auto referenceManager = std::make_shared<ReferenceManager>(std::move(targetTrajectories));

  // operatingTrajectories
  const vector_t stateOperatingPoint = vector_t::Zero(STATE_DIM);
  const vector_t inputOperatingPoint = vector_t::Zero(INPUT_DIM);
  OperatingPoints operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  // Test 1: Check constraint compliance. It uses a solver observer to get metrics for the bounds constraints
  auto boundsConstraintsObserverPtr = SolverObserver::LagrangianTermObserver(
      SolverObserver::Type::Intermediate, "bounds",
      [&](const scalar_array_t& timeTraj, const std::vector<LagrangianMetricsConstRef>& metricsTraj) {
        constexpr scalar_t constraintViolationTolerance = 1e-1;
        for (size_t i = 0; i < metricsTraj.size(); i++) {
          const vector_t constraintViolation = metricsTraj[i].constraint.cwiseMin(0.0);
          EXPECT_NEAR(constraintViolation(0), 0.0, constraintViolationTolerance) << "At time " << timeTraj[i] << "\n";
          EXPECT_NEAR(constraintViolation(1), 0.0, constraintViolationTolerance) << "At time " << timeTraj[i] << "\n";
          EXPECT_NEAR(constraintViolation(2), 0.0, constraintViolationTolerance) << "At time " << timeTraj[i] << "\n";
          EXPECT_NEAR(constraintViolation(3), 0.0, constraintViolationTolerance) << "At time " << timeTraj[i] << "\n";
        }
      });

  // setup SLQ
  SLQ slq(ddpSettings, stateTriggeredRollout, problem, operatingTrajectories);
  slq.SetReferenceManager(referenceManager);
  slq.AddSolverObserver(std::move(boundsConstraintsObserverPtr));

  // run SLQ
  slq.Run(startTime, initState, finalTime);
  const auto solution = slq.PrimalSolution(finalTime);

  // Test 2: No penetration of guard surfaces
  for (int i = 0; i < solution.timeTrajectory_.size(); i++) {
    const vector_t guardSurfaces = systemDynamics.ComputeGuardSurfaces(solution.timeTrajectory_[i], solution.stateTrajectory_[i]);
    EXPECT_GT(GuardSurfaces(0), -1e-10) << solution.timeTrajectory_[i] << ": " << GuardSurfaces(0) << ", " << GuardSurfaces(1) << "\n";
    EXPECT_GT(GuardSurfaces(1), -1e-10) << solution.timeTrajectory_[i] << ": " << GuardSurfaces(0) << ", " << GuardSurfaces(1) << "\n";
  }

  // Test 3: Check of cost
  const auto performanceIndecesST = slq.GetPerformanceIndeces();
  EXPECT_LT(performanceIndecesST.cost - 20.1, 10.0 * minRelCost);
}
