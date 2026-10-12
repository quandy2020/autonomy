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

#include <bolza/oc/multiple_shooting/transcription.hpp>
#include <bolza/oc/multiple_shooting/performance_index_computation.hpp>

#include "helpers/circular_kinematics.hpp"
#include "helpers/problems_generation_test.hpp"

using namespace bolza;

TEST(test_transcription_performance, intermediate) {
  constexpr int nx = 2;
  constexpr int nu = 2;

  // optimal control problem
  OptimalControlProblem problem = CreateCircularKinematicsProblem("/tmp/sqp_test_generated");

  // equality constraints
  problem.equalityConstraintPtr->Add("equalityConstraint", GetOcs2Constraints(GetRandomConstraints(nx, nu, 2)));
  problem.stateEqualityConstraintPtr->Add("stateEqualityConstraint", GetOcs2StateOnlyConstraints(GetRandomConstraints(nx, 0, 1)));
  // inequality constraints
  problem.inequalityConstraintPtr->Add("inequalityConstraint", GetOcs2Constraints(GetRandomConstraints(nx, nu, 3)));
  problem.stateInequalityConstraintPtr->Add("stateInequalityConstraint", GetOcs2StateOnlyConstraints(GetRandomConstraints(nx, 0, 4)));

  auto discretizer = SelectDynamicsDiscretization(SensitivityIntegratorType::RK4);
  auto sensitivityDiscretizer = SelectDynamicsSensitivityDiscretization(SensitivityIntegratorType::RK4);

  const scalar_t t = 0.5;
  const scalar_t dt = 0.1;
  const vector_t x = (vector_t(nx) << 1.0, 0.1).finished();
  const vector_t x_next = (vector_t(nx) << 1.1, 0.2).finished();
  const vector_t u = (vector_t(nu) << 0.1, 1.3).finished();
  const auto transcription = multiple_shooting::SetupIntermediateNode(problem, sensitivityDiscretizer, t, dt, x, x_next, u);
  const auto performance = multiple_shooting::ComputeIntermediatePerformance(problem, discretizer, t, dt, x, x_next, u);

  ASSERT_TRUE(performance.isApprox(multiple_shooting::ComputePerformanceIndex(transcription, dt), 1e-12));
}

TEST(test_transcription_performance, event) {
  constexpr int nx = 2;

  // optimal control problem
  OptimalControlProblem problem;

  // dynamics
  const auto dynamics = GetRandomDynamics(nx, 0);
  const auto jumpMap = matrix_t::Random(nx, nx);
  problem.dynamicsPtr.Reset(new LinearSystemDynamics(dynamics.dfdx, dynamics.dfdu, jumpMap));

  // cost
  problem.preJumpCostPtr->Add("eventCost", GetOcs2StateCost(GetRandomCost(nx, 0)));

  // constraints
  problem.preJumpEqualityConstraintPtr->Add("preJumpEqualityConstraint", GetOcs2StateOnlyConstraints(GetRandomConstraints(nx, 0, 3)));
  problem.preJumpInequalityConstraintPtr->Add("preJumpInequalityConstraint", GetOcs2StateOnlyConstraints(GetRandomConstraints(nx, 0, 4)));

  const TargetTrajectories TargetTrajectories({0.0}, {vector_t::Random(nx)}, {vector_t::Random(0)});
  problem.targetTrajectoriesPtr = &targetTrajectories;

  const scalar_t t = 0.5;
  const vector_t x = (vector_t(nx) << 1.0, 0.1).finished();
  const vector_t x_next = (vector_t(nx) << 1.1, 0.2).finished();
  const auto transcription = multiple_shooting::SetupEventNode(problem, t, x, x_next);
  const auto performance = multiple_shooting::ComputeEventPerformance(problem, t, x, x_next);

  ASSERT_TRUE(performance.isApprox(multiple_shooting::ComputePerformanceIndex(transcription), 1e-12));
}

TEST(test_transcription_performance, terminal) {
  constexpr int nx = 3;

  // optimal control problem
  OptimalControlProblem problem;

  // cost
  problem.finalCostPtr->Add("finalCost", GetOcs2StateCost(GetRandomCost(nx, 0)));
  problem.finalSoftConstraintPtr->Add("finalSoftCost", GetOcs2StateCost(GetRandomCost(nx, 0)));

  // constraints
  problem.finalEqualityConstraintPtr->Add("finalEqualityConstraint", GetOcs2StateOnlyConstraints(GetRandomConstraints(nx, 0, 3)));
  problem.finalInequalityConstraintPtr->Add("finalInequalityConstraint", GetOcs2StateOnlyConstraints(GetRandomConstraints(nx, 0, 4)));

  const TargetTrajectories TargetTrajectories({0.0}, {vector_t::Random(nx)}, {vector_t::Random(0)});
  problem.targetTrajectoriesPtr = &targetTrajectories;

  const scalar_t t = 0.5;
  const vector_t x = vector_t::Random(nx);
  const auto transcription = multiple_shooting::SetupTerminalNode(problem, t, x);
  const auto performance = multiple_shooting::ComputeTerminalPerformance(problem, t, x);

  ASSERT_TRUE(performance.isApprox(multiple_shooting::ComputePerformanceIndex(transcription), 1e-12));
}
