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

#include "bolza/oc/oc_problem/ocp_size.hpp"
#include "bolza/oc/oc_problem/ocp_to_kkt.hpp"

#include "helpers/problems_generation_test.hpp"

class OcpToKktTest : public testing::Test {
 protected:
  // x_0, x_1, ... x_{N - 1}, X_{N}
  static constexpr size_t N_ = 10;  // numStages
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t nc_ = 5;
  static constexpr size_t numDecisionVariables = N_ * (nx_ + nu_);
  static constexpr size_t numConstraints = N_ * (nx_ + nc_);

  OcpToKktTest() {
    srand(0);

    x0 = bolza::vector_t::Random(nx_);
    for (int i = 0; i < N_; i++) {
      dynamicsArray.push_back(bolza::GetRandomDynamics(nx_, nu_));
      costArray.push_back(bolza::GetRandomCost(nx_, nu_));
      constraintsArray.push_back(bolza::GetRandomConstraints(nx_, nu_, nc_));
    }
    costArray.push_back(bolza::GetRandomCost(nx_, nu_));
    constraintsArray.push_back(bolza::GetRandomConstraints(nx_, nu_, nc_));

    ocpSize_ = bolza::ExtractSizesFromProblem(dynamicsArray, costArray, &constraintsArray);
  }

  bolza::OcpSize ocpSize_;
  bolza::vector_t x0;
  std::vector<bolza::VectorFunctionLinearApproximation> dynamicsArray;
  std::vector<bolza::ScalarFunctionQuadraticApproximation> costArray;
  std::vector<bolza::VectorFunctionLinearApproximation> constraintsArray;
};

TEST_F(OcpToKktTest, sparseConstraintsApproximation) {
  Eigen::SparseMatrix<bolza::scalar_t> G;
  bolza::vector_t g;
  bolza::vector_array_t scalingVectors(N_);
  for (auto& v : scalingVectors) {
    v = bolza::vector_t::Random(nx_);
  }
  bolza::VectorFunctionLinearApproximation constraintsApproximation;
  bolza::GetConstraintMatrix(ocpSize_, x0, dynamicsArray, &constraintsArray, &scalingVectors, constraintsApproximation);
  bolza::GetConstraintMatrixSparse(ocpSize_, x0, dynamicsArray, &constraintsArray, &scalingVectors, G, g);

  EXPECT_TRUE(constraintsApproximation.dfdx.isApprox(G.toDense()));
  EXPECT_TRUE(constraintsApproximation.f.isApprox(g));
}

TEST_F(OcpToKktTest, sparseCostApproximation) {
  Eigen::SparseMatrix<bolza::scalar_t> H;
  bolza::vector_t h;

  bolza::ScalarFunctionQuadraticApproximation costApproximation;
  bolza::GetCostMatrix(ocpSize_, x0, costArray, costApproximation);
  bolza::GetCostMatrixSparse(ocpSize_, x0, costArray, H, h);

  EXPECT_TRUE(costApproximation.dfdxx.isApprox(H.toDense()));
  EXPECT_TRUE(costApproximation.dfdx.isApprox(h));
}
