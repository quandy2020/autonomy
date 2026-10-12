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

#include "bolza/qp/qp_discrete_transcription.hpp"
#include "helpers/problems_generation_test.hpp"

class DiscreteTranscriptionTest : public testing::Test {
 protected:
  static constexpr size_t N = 10;  // Trajectory length
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  static constexpr size_t numStateInputConstraints = 1;
  static constexpr size_t numStateOnlyConstraints = 1;
  static constexpr size_t numFinalStateOnlyConstraints = 1;
  static constexpr bolza::scalar_t dt = 1e-2;

  DiscreteTranscriptionTest() {
    srand(0);

    targetTrajectories = bolza::TargetTrajectories({0.0}, {bolza::vector_t::Random(STATE_DIM)}, {bolza::vector_t::Random(INPUT_DIM)});

    system = bolza::GetOcs2Dynamics(bolza::GetRandomDynamics(STATE_DIM, INPUT_DIM));

    bolza::OptimalControlProblem problem;
    problem.dynamicsPtr.reset(system->Clone());

    problem.costPtr->Add("IntermediateCost", bolza::GetOcs2Cost(bolza::GetRandomCost(STATE_DIM, INPUT_DIM)));
    problem.finalCostPtr->Add("FinalCost", bolza::GetOcs2StateCost(bolza::GetRandomCost(STATE_DIM, 0)));
    problem.targetTrajectoriesPtr = &targetTrajectories;

    linearization = bolza::qp_solver::GetRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);

    unconstrainedLqr = bolza::qp_solver::GetLinearQuadraticApproximation(problem, linearization);

    constrainedProblem = problem;  // copies unconstrained problem
    constrainedProblem.targetTrajectoriesPtr = &targetTrajectories;

    constrainedProblem.equalityConstraintPtr->Add(
        "equality", bolza::GetOcs2Constraints(bolza::GetRandomConstraints(STATE_DIM, INPUT_DIM, numStateInputConstraints)));
    constrainedProblem.stateEqualityConstraintPtr->Add(
        "equality", bolza::GetOcs2StateOnlyConstraints(bolza::GetRandomConstraints(STATE_DIM, 0, numStateOnlyConstraints)));
    constrainedProblem.finalEqualityConstraintPtr->Add(
        "equality", bolza::GetOcs2StateOnlyConstraints(bolza::GetRandomConstraints(STATE_DIM, 0, numFinalStateOnlyConstraints)));

    constrainedLqr = bolza::qp_solver::GetLinearQuadraticApproximation(constrainedProblem, linearization);
  }

  void checkSizes(const std::vector<bolza::qp_solver::LinearQuadraticStage>& lqr, size_t numStateInputConstraints,
                  size_t numStateOnlyConstraints, size_t numTerminalConstraints) const {
    ASSERT_EQ(lqr.size(), N + 1);
    for (int k = 0; k < N; ++k) {
      // Cost sizes
      ASSERT_EQ(lqr[k].cost.dfdxx.rows(), STATE_DIM);
      ASSERT_EQ(lqr[k].cost.dfdxx.cols(), STATE_DIM);
      ASSERT_EQ(lqr[k].cost.dfdux.rows(), INPUT_DIM);
      ASSERT_EQ(lqr[k].cost.dfdux.cols(), STATE_DIM);
      ASSERT_EQ(lqr[k].cost.dfduu.rows(), INPUT_DIM);
      ASSERT_EQ(lqr[k].cost.dfduu.cols(), INPUT_DIM);

      // Dynamics sizes
      ASSERT_EQ(lqr[k].dynamics.dfdx.rows(), STATE_DIM);
      ASSERT_EQ(lqr[k].dynamics.dfdx.cols(), STATE_DIM);
      ASSERT_EQ(lqr[k].dynamics.dfdu.rows(), STATE_DIM);
      ASSERT_EQ(lqr[k].dynamics.dfdu.cols(), INPUT_DIM);

      // Constraint sizes
      const auto numIntermediateConstraints = k == 0 ? numStateInputConstraints : numStateInputConstraints + numStateOnlyConstraints;
      ASSERT_EQ(lqr[k].constraints.f.rows(), numIntermediateConstraints);
      ASSERT_EQ(lqr[k].constraints.dfdx.rows(), numIntermediateConstraints);
      ASSERT_EQ(lqr[k].constraints.dfdu.rows(), numIntermediateConstraints);
      if (numIntermediateConstraints > 0) {
        ASSERT_EQ(lqr[k].constraints.dfdx.cols(), STATE_DIM);
        ASSERT_EQ(lqr[k].constraints.dfdu.cols(), INPUT_DIM);
      }
    }

    // Terminal Cost size
    ASSERT_EQ(lqr[N].cost.dfdxx.rows(), STATE_DIM);
    ASSERT_EQ(lqr[N].cost.dfdxx.cols(), STATE_DIM);

    // Terminal Constraint size
    ASSERT_EQ(lqr[N].constraints.f.rows(), numTerminalConstraints);
    ASSERT_EQ(lqr[N].constraints.dfdx.rows(), numTerminalConstraints);
    if (numTerminalConstraints > 0) {
      ASSERT_EQ(lqr[N].constraints.dfdx.cols(), STATE_DIM);
    }
  }

  bolza::TargetTrajectories targetTrajectories;
  std::unique_ptr<bolza::SystemDynamicsBase> system;
  bolza::OptimalControlProblem constrainedProblem;
  bolza::qp_solver::ContinuousTrajectory linearization;
  std::vector<bolza::qp_solver::LinearQuadraticStage> unconstrainedLqr;
  std::vector<bolza::qp_solver::LinearQuadraticStage> constrainedLqr;
};

constexpr size_t DiscreteTranscriptionTest::N;
constexpr size_t DiscreteTranscriptionTest::STATE_DIM;
constexpr size_t DiscreteTranscriptionTest::INPUT_DIM;
constexpr size_t DiscreteTranscriptionTest::numStateInputConstraints;
constexpr size_t DiscreteTranscriptionTest::numStateOnlyConstraints;
constexpr size_t DiscreteTranscriptionTest::numFinalStateOnlyConstraints;
constexpr bolza::scalar_t DiscreteTranscriptionTest::dt;

TEST_F(DiscreteTranscriptionTest, unconstrainedLqrHasCorrectSizes) {
  checkSizes(unconstrainedLqr, 0, 0, 0);
}

TEST_F(DiscreteTranscriptionTest, constrainedLqrHasCorrectSizes) {
  checkSizes(constrainedLqr, numStateInputConstraints, numStateOnlyConstraints, numFinalStateOnlyConstraints);
}

TEST_F(DiscreteTranscriptionTest, linearizationInvariance) {
  auto linearization2 = bolza::qp_solver::GetRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
  linearization2.timeTrajectory = linearization.timeTrajectory;

  const auto lqp2 = bolza::qp_solver::GetLinearQuadraticApproximation(constrainedProblem, linearization2);

  // All matrices should stay the same. The linear and constant parts changes
  for (int k = 0; k < N; ++k) {
    // Cost
    ASSERT_TRUE(constrainedLqr[k].cost.dfdxx.isApprox(lqp2[k].cost.dfdxx));
    ASSERT_TRUE(constrainedLqr[k].cost.dfdux.isApprox(lqp2[k].cost.dfdux));
    ASSERT_TRUE(constrainedLqr[k].cost.dfduu.isApprox(lqp2[k].cost.dfduu));

    // Dynamics
    ASSERT_TRUE(constrainedLqr[k].dynamics.dfdx.isApprox(lqp2[k].dynamics.dfdx));
    ASSERT_TRUE(constrainedLqr[k].dynamics.dfdu.isApprox(lqp2[k].dynamics.dfdu));

    // Constraints
    ASSERT_TRUE(constrainedLqr[k].constraints.dfdx.isApprox(lqp2[k].constraints.dfdx));
    ASSERT_TRUE(constrainedLqr[k].constraints.dfdu.isApprox(lqp2[k].constraints.dfdu));
  }

  // Terminal Cost
  ASSERT_TRUE(constrainedLqr[N].cost.dfdxx.isApprox(lqp2[N].cost.dfdxx));

  // Terminal Constraints
  ASSERT_TRUE(constrainedLqr[N].constraints.dfdx.isApprox(lqp2[N].constraints.dfdx));
}
