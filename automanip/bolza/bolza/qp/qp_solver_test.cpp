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

#include "bolza/qp/qp_solver.hpp"
#include "helpers/problems_generation_test.hpp"

class QpSolverTest : public testing::Test {
 protected:
  static constexpr size_t N_ = 5;
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t nc_ = 2;
  static constexpr size_t numDecisionVariables = N_ * (nx_ + nu_) + nx_;
  static constexpr size_t numConstraints = (N_ + 1) * (nx_ + nc_);

  QpSolverTest() {
    srand(0);
    lqProblem = bolza::qp_solver::GenerateRandomLqProblem(N_, nx_, nu_, nc_);
    x0 = bolza::vector_t::Random(nx_);

    cost = bolza::qp_solver::GetCostMatrices(lqProblem, numDecisionVariables);
    constraints = bolza::qp_solver::GetConstraintMatrices(lqProblem, x0, numConstraints, numDecisionVariables);
    std::tie(primalSolution, dualSolution) = bolza::qp_solver::SolveDenseQp(cost, constraints);
  }

  std::vector<bolza::qp_solver::LinearQuadraticStage> lqProblem;
  bolza::vector_t x0;
  bolza::ScalarFunctionQuadraticApproximation cost;
  bolza::VectorFunctionLinearApproximation constraints;
  bolza::vector_t primalSolution;
  bolza::vector_t dualSolution;
};

constexpr size_t QpSolverTest::N_;
constexpr size_t QpSolverTest::nx_;
constexpr size_t QpSolverTest::nu_;
constexpr size_t QpSolverTest::nc_;
constexpr size_t QpSolverTest::numDecisionVariables;
constexpr size_t QpSolverTest::numConstraints;

TEST_F(QpSolverTest, constraintSatisfaction) {
  ASSERT_TRUE(constraints.f.isApprox(-constraints.dfdx * primalSolution));
}

TEST_F(QpSolverTest, firstOrderOptimality) {
  ASSERT_TRUE(cost.dfdx.isApprox(-cost.dfdxx * primalSolution - constraints.dfdx.transpose() * dualSolution));
}

TEST_F(QpSolverTest, costMatrixFullRank) {
  ASSERT_TRUE(cost.dfdxx.fullPivLu().Rank() == cost.dfdxx.rows());
}

TEST_F(QpSolverTest, constraintMatrixFullRank) {
  ASSERT_TRUE(constraints.dfdx.fullPivLu().Rank() == constraints.dfdx.rows());
}
