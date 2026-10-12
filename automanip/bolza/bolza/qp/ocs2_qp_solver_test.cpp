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

#include <bolza/oc/oc_problem/optimal_control_problem.hpp>

#include <bolza/core/cost/quadratic_state_cost.hpp>
#include <bolza/core/cost/quadratic_state_input_cost.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>
#include <bolza/core/model_data/metrics.hpp>

#include "helpers/tools_test.hpp"

#include "bolza/qp/qp_solver.hpp"
#include "helpers/problems_generation_test.hpp"

class Ocs2QpSolverTest : public testing::Test {
 protected:
  static constexpr size_t N = 10;  // Trajectory length
  static constexpr size_t STATE_DIM = 5;
  static constexpr size_t INPUT_DIM = 3;
  static constexpr size_t numStateInputConstraints = 1;
  static constexpr size_t numStateOnlyConstraints = 1;
  static constexpr size_t numFinalStateOnlyConstraints = 2;
  static constexpr bolza::scalar_t precision = 1e-9;
  static constexpr bolza::scalar_t dt = 1e-3;

  void checkDynamics(const bolza::qp_solver::ContinuousTrajectory& solution) const {
    // Forward integrate with unconstrainedSolution u(t) and check x(t)
    bolza::vector_t x = x0;
    for (int k = 0; k < N; ++k) {
      auto dt = solution.timeTrajectory[k + 1] - solution.timeTrajectory[k];
      x += dt * system->ComputeFlowMap(solution.timeTrajectory[k], x, solution.inputTrajectory[k]);
      ASSERT_TRUE(x.isApprox(solution.stateTrajectory[k + 1], precision));
    }
  }

  Ocs2QpSolverTest() {
    srand(0);

    system = bolza::GetOcs2Dynamics(bolza::GetRandomDynamics(STATE_DIM, INPUT_DIM));

    // create unconstrained problem
    unconstrainedProblem = bolza::OptimalControlProblem();  // reset
    unconstrainedProblem.dynamicsPtr.reset(system->Clone());

    unconstrainedProblem.costPtr->Add("IntermediateCost", bolza::GetOcs2Cost(bolza::GetRandomCost(STATE_DIM, INPUT_DIM)));
    unconstrainedProblem.finalCostPtr->Add("FinalCost", bolza::GetOcs2StateCost(bolza::GetRandomCost(STATE_DIM, 0)));

    // create constrained problem
    constrainedProblem = unconstrainedProblem;  // copies unconstrained problem

    constrainedProblem.equalityConstraintPtr->Add(
        "equality", bolza::GetOcs2Constraints(bolza::GetRandomConstraints(STATE_DIM, INPUT_DIM, numStateInputConstraints)));
    constrainedProblem.stateEqualityConstraintPtr->Add(
        "equality", bolza::GetOcs2StateOnlyConstraints(bolza::GetRandomConstraints(STATE_DIM, 0, numStateOnlyConstraints)));
    constrainedProblem.finalEqualityConstraintPtr->Add(
        "equality", bolza::GetOcs2StateOnlyConstraints(bolza::GetRandomConstraints(STATE_DIM, 0, numFinalStateOnlyConstraints)));

    targetTrajectories = bolza::TargetTrajectories({0.0}, {bolza::vector_t::Random(STATE_DIM)}, {bolza::vector_t::Random(INPUT_DIM)});
    constrainedProblem.targetTrajectoriesPtr = &targetTrajectories;
    unconstrainedProblem.targetTrajectoriesPtr = &targetTrajectories;

    nominalTrajectory = bolza::qp_solver::GetRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
    x0 = bolza::vector_t::Random(STATE_DIM);
    unconstrainedSolution = SolveLinearQuadraticOptimalControlProblem(unconstrainedProblem, nominalTrajectory, x0);
    constrainedSolution = SolveLinearQuadraticOptimalControlProblem(constrainedProblem, nominalTrajectory, x0);
  }

  bolza::TargetTrajectories targetTrajectories;
  std::unique_ptr<bolza::SystemDynamicsBase> system;
  bolza::OptimalControlProblem unconstrainedProblem;
  bolza::OptimalControlProblem constrainedProblem;
  bolza::qp_solver::ContinuousTrajectory nominalTrajectory;
  bolza::vector_t x0;
  bolza::qp_solver::ContinuousTrajectory constrainedSolution;
  bolza::qp_solver::ContinuousTrajectory unconstrainedSolution;
};

constexpr size_t Ocs2QpSolverTest::N;
constexpr size_t Ocs2QpSolverTest::STATE_DIM;
constexpr size_t Ocs2QpSolverTest::INPUT_DIM;
constexpr size_t Ocs2QpSolverTest::numStateInputConstraints;
constexpr size_t Ocs2QpSolverTest::numStateOnlyConstraints;
constexpr size_t Ocs2QpSolverTest::numFinalStateOnlyConstraints;
constexpr bolza::scalar_t Ocs2QpSolverTest::precision;
constexpr bolza::scalar_t Ocs2QpSolverTest::dt;

TEST_F(Ocs2QpSolverTest, initialCondition) {
  ASSERT_TRUE(x0.isApprox(constrainedSolution.stateTrajectory.front(), precision));
  ASSERT_TRUE(x0.isApprox(unconstrainedSolution.stateTrajectory.front(), precision));
}

TEST_F(Ocs2QpSolverTest, satisfiesDynamics) {
  checkDynamics(constrainedSolution);
  checkDynamics(unconstrainedSolution);
}

TEST_F(Ocs2QpSolverTest, satisfiesConstraints) {
  auto& preComputation = *unconstrainedProblem.preComputationPtr;
  const auto t0 = constrainedSolution.timeTrajectory[0];
  const auto& x0 = constrainedSolution.stateTrajectory[0];
  const auto& u0 = constrainedSolution.inputTrajectory[0];
  preComputation.Request(bolza::Request::Constraint, t0, x0, u0);

  const auto g0 = bolza::ToVector(unconstrainedProblem.equalityConstraintPtr->GetValue(t0, x0, u0, preComputation));
  ASSERT_TRUE(g0.isZero(precision));

  for (int k = 1; k < N; ++k) {
    const auto t = constrainedSolution.timeTrajectory[k];
    const auto& x = constrainedSolution.stateTrajectory[k];
    const auto& u = constrainedSolution.inputTrajectory[k];
    preComputation.Request(bolza::Request::Constraint, t, x, u);

    const auto g = bolza::ToVector(unconstrainedProblem.equalityConstraintPtr->GetValue(t, x, u, preComputation));
    ASSERT_TRUE(g.isZero(precision));

    const auto h = bolza::ToVector(unconstrainedProblem.stateEqualityConstraintPtr->GetValue(t, x, preComputation));
    ASSERT_TRUE(h.isZero(precision));
  }

  const auto tf = constrainedSolution.timeTrajectory[N];
  const auto& xf = constrainedSolution.stateTrajectory[N];
  preComputation.RequestFinal(bolza::Request::Constraint, tf, xf);

  const auto gf = bolza::ToVector(unconstrainedProblem.finalEqualityConstraintPtr->GetValue(tf, xf, preComputation));
  ASSERT_TRUE(gf.isZero(precision));
}

TEST_F(Ocs2QpSolverTest, invariantUnderLinearization) {
  // Different nominalTrajectory, with same time discretization
  auto linearization2 = bolza::qp_solver::GetRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
  linearization2.timeTrajectory = nominalTrajectory.timeTrajectory;

  // Compare solutions
  auto solution2 = SolveLinearQuadraticOptimalControlProblem(constrainedProblem, linearization2, x0);
  ASSERT_TRUE(bolza::IsEqual(constrainedSolution.stateTrajectory, solution2.stateTrajectory, precision));
  ASSERT_TRUE(bolza::IsEqual(constrainedSolution.inputTrajectory, solution2.inputTrajectory, precision));
}

TEST_F(Ocs2QpSolverTest, knownSolutionAtOrigin) {
  // If the cost's nominal trajectory is set to zero, and the initial state is zero, then the solution has only zeros.
  targetTrajectories = bolza::TargetTrajectories({0.0}, {bolza::vector_t::Zero(STATE_DIM)}, {bolza::vector_t::Zero(INPUT_DIM)});
  unconstrainedProblem.targetTrajectoriesPtr = &targetTrajectories;
  const auto zeroX0 = bolza::vector_t::Zero(STATE_DIM);

  // Obtain solution, with non-zero nominalTrajectory
  auto zeroSolution = SolveLinearQuadraticOptimalControlProblem(unconstrainedProblem, nominalTrajectory, zeroX0);

  bolza::vector_array_t allStatesZero(N + 1, bolza::vector_t::Zero(STATE_DIM));
  bolza::vector_array_t allInputsZero(N, bolza::vector_t::Zero(INPUT_DIM));
  ASSERT_TRUE(bolza::IsEqual(zeroSolution.stateTrajectory, allStatesZero));
  ASSERT_TRUE(bolza::IsEqual(zeroSolution.inputTrajectory, allInputsZero));
}
