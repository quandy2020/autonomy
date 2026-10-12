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

#include <bolza/oc/oc_problem/ocp_to_kkt.hpp>
#include "helpers/problems_generation_test.hpp"

#include "bolza/slp/helpers.hpp"

class HelperFunctionTest : public testing::Test {
 protected:
  // x_0, x_1, ... x_{N - 1}, X_{N}
  static constexpr size_t N_ = 10;  // numStages
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t nc_ = 0;
  static constexpr size_t numDecisionVariables = N_ * (nx_ + nu_);
  static constexpr size_t numConstraints = N_ * (nx_ + nc_);
  static constexpr bool verbose = true;

  HelperFunctionTest() : threadPool_(5, 99) {
    srand(10);

    // Construct OCP problem
    x0 = bolza::vector_t::Random(nx_);

    for (int i = 0; i < N_; i++) {
      dynamicsArray.push_back(bolza::GetRandomDynamics(nx_, nu_));
      costArray.push_back(bolza::GetRandomCost(nx_, nu_));
      constraintsArray.push_back(bolza::GetRandomConstraints(nx_, nu_, nc_));
    }
    costArray.push_back(bolza::GetRandomCost(nx_, nu_));
    constraintsArray.push_back(bolza::GetRandomConstraints(nx_, nu_, nc_));
    ocpSize_ = bolza::ExtractSizesFromProblem(dynamicsArray, costArray, nullptr);

    bolza::GetCostMatrix(ocpSize_, x0, costArray, costApproximation);
  }

  bolza::OcpSize ocpSize_;
  bolza::vector_t x0;
  bolza::ScalarFunctionQuadraticApproximation costApproximation;

  std::vector<bolza::VectorFunctionLinearApproximation> dynamicsArray;
  std::vector<bolza::ScalarFunctionQuadraticApproximation> costArray;
  std::vector<bolza::VectorFunctionLinearApproximation> constraintsArray;

  bolza::ThreadPool threadPool_;
};

TEST_F(HelperFunctionTest, hessianAbsRowSum) {
  bolza::vector_t rowwiseSum = bolza::slp::HessianAbsRowSum(ocpSize_, costArray);
  EXPECT_TRUE(rowwiseSum.isApprox(costApproximation.dfdxx.cwiseAbs().rowwise().sum())) << "rowSum:\n" << rowwiseSum.transpose();
}

TEST_F(HelperFunctionTest, GGTAbsRowSumInParallel) {
  bolza::VectorFunctionLinearApproximation constraintsApproximation;
  bolza::vector_array_t scalingVectors(N_);
  for (auto& v : scalingVectors) {
    v = bolza::vector_t::Random(nx_);
  }

  bolza::GetConstraintMatrix(ocpSize_, x0, dynamicsArray, nullptr, &scalingVectors, constraintsApproximation);
  bolza::vector_t rowwiseSum = bolza::slp::GGTAbsRowSumInParallel(threadPool_, ocpSize_, dynamicsArray, nullptr, &scalingVectors);
  bolza::matrix_t GGT = constraintsApproximation.dfdx * constraintsApproximation.dfdx.transpose();
  EXPECT_TRUE(rowwiseSum.isApprox(GGT.cwiseAbs().rowwise().sum()));
}