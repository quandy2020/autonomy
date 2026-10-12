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

#include <bolza/core/thread_support/thread_pool.hpp>

#include "bolza/oc/oc_problem/ocp_to_kkt.hpp"
#include "bolza/oc/precondition/ruzi.hpp"
#include "helpers/problems_generation_test.hpp"

class PreconditionTest : public testing::Test {
 protected:
  // x_0, x_1, ... x_{N - 1}, X_{N}
  static constexpr size_t N_ = 10;  // numStages
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t numDecisionVariables_ = N_ * (nx_ + nu_);
  static constexpr size_t numConstraints_ = N_ * nx_;

  PreconditionTest() {
    srand(0);

    x0 = bolza::vector_t::Random(nx_);
    for (int i = 0; i < N_; i++) {
      dynamicsArray.push_back(bolza::GetRandomDynamics(nx_, nu_));
      costArray.push_back(bolza::GetRandomCost(nx_, nu_));
    }
    costArray.push_back(bolza::GetRandomCost(nx_, 0));

    ocpSize_ = bolza::ExtractSizesFromProblem(dynamicsArray, costArray, nullptr);
  }

  bolza::OcpSize ocpSize_;
  bolza::vector_t x0;
  std::vector<bolza::VectorFunctionLinearApproximation> dynamicsArray;
  std::vector<bolza::ScalarFunctionQuadraticApproximation> costArray;
};

constexpr size_t PreconditionTest::N_;
constexpr size_t PreconditionTest::nx_;
constexpr size_t PreconditionTest::nu_;
constexpr size_t PreconditionTest::numDecisionVariables_;
constexpr size_t PreconditionTest::numConstraints_;

TEST_F(PreconditionTest, kktMatrixInPlace) {
  bolza::vector_t D, E;
  bolza::scalar_t c;

  Eigen::SparseMatrix<bolza::scalar_t> H;
  bolza::vector_t h;
  bolza::GetCostMatrixSparse(ocpSize_, x0, costArray, H, h);
  // Copy of the original matrix/vector
  const Eigen::SparseMatrix<bolza::scalar_t> H_src = H;
  const bolza::vector_t h_src = h;

  Eigen::SparseMatrix<bolza::scalar_t> G;
  bolza::vector_t g;
  bolza::GetConstraintMatrixSparse(ocpSize_, x0, dynamicsArray, nullptr, nullptr, G, g);
  // Copy of the original matrix/vector
  const Eigen::SparseMatrix<bolza::scalar_t> G_src = G;
  const bolza::vector_t g_src = g;

  // Test 1: Construct the stacked cost and constraints matrices first and scale next.
  bolza::precondition::KktMatrixInPlace(5, H, h, G, g, D, E, c);

  // After pre-conditioning, H, h, G, and g will be scaled in place..
  const Eigen::SparseMatrix<bolza::scalar_t> H_ref = c * D.asDiagonal() * H_src * D.asDiagonal();
  const bolza::vector_t h_ref = c * D.asDiagonal() * h_src;
  const Eigen::SparseMatrix<bolza::scalar_t> G_ref = E.asDiagonal() * G_src * D.asDiagonal();
  const bolza::vector_t g_ref = E.asDiagonal() * g_src;

  // The scaled matrix given by calculatePreConditioningFactors function and the one calculated with D, E and c factors should be the same.
  ASSERT_TRUE(H_ref.isApprox(H));                                                    // H
  ASSERT_TRUE(h_ref.isApprox(h));                                                    // h
  ASSERT_TRUE(G_ref.isApprox(G)) << "G_ref:\n" << G_ref << "\nG:\n" << G.toDense();  // G
  ASSERT_TRUE(g_ref.isApprox(g));                                                    // g

  // Normalized the inf-Norm of both rows and cols of KKT matrix should be closer to 1
  const bolza::matrix_t KKT = (bolza::matrix_t(H_src.rows() + G_src.rows(), H_src.rows() + G_src.rows()) << H_src.toDense(),
                              G_src.transpose().toDense(), G_src.toDense(), bolza::matrix_t::Zero(G_src.rows(), G_src.rows()))
                                 .finished();

  const bolza::matrix_t KKTScaled = (bolza::matrix_t(H.rows() + G.rows(), H.rows() + G.rows()) << H.toDense(), G.transpose().toDense(),
                                    G.toDense(), bolza::matrix_t::Zero(G.rows(), G.rows()))
                                       .finished();

  // Inf-norm of row vectors
  bolza::vector_t infNormRows = KKT.cwiseAbs().rowwise().maxCoeff();
  bolza::vector_t infNormRowsScaled = KKTScaled.cwiseAbs().rowwise().maxCoeff();
  EXPECT_LT((infNormRowsScaled.array() - 1).abs().maxCoeff(), (infNormRows.array() - 1).abs().maxCoeff());
  // Inf-norm of column vectors
  bolza::vector_t infNormCols = KKT.cwiseAbs().colwise().maxCoeff();
  bolza::vector_t infNormColsScaled = KKTScaled.cwiseAbs().colwise().maxCoeff();
  EXPECT_LT((infNormColsScaled.array() - 1).abs().maxCoeff(), (infNormCols.array() - 1).abs().maxCoeff());

  // Test 2: Scale const and dynamics data of each time step first and then construct the stacked matrices from the scaled data.
  std::vector<bolza::vector_t> scalingVectors;
  bolza::precondition::ScaleOcpData(ocpSize_, D, E, c, dynamicsArray, costArray, scalingVectors);

  Eigen::SparseMatrix<bolza::scalar_t> H_scaledData;
  bolza::vector_t h_scaledData;
  bolza::GetCostMatrixSparse(ocpSize_, x0, costArray, H_scaledData, h_scaledData);
  Eigen::SparseMatrix<bolza::scalar_t> G_scaledData;
  bolza::vector_t g_scaledData;
  bolza::GetConstraintMatrixSparse(ocpSize_, x0, dynamicsArray, nullptr, &scalingVectors, G_scaledData, g_scaledData);
  EXPECT_TRUE(H_ref.isApprox(H_scaledData));  // H
  EXPECT_TRUE(h_ref.isApprox(h_scaledData));  // h
  EXPECT_TRUE(G_ref.isApprox(G_scaledData));  // G
  EXPECT_TRUE(g_ref.isApprox(g_scaledData));  // g
}

TEST_F(PreconditionTest, ocpDataInPlaceInParallel) {
  bolza::ThreadPool threadPool(5, 99);

  bolza::vector_t D_ref, E_ref;
  bolza::scalar_t c_ref;

  Eigen::SparseMatrix<bolza::scalar_t> H_ref;
  bolza::vector_t h_ref;
  bolza::GetCostMatrixSparse(ocpSize_, x0, costArray, H_ref, h_ref);

  // Generate reference
  bolza::vector_t g_ref;
  Eigen::SparseMatrix<bolza::scalar_t> G_ref;
  bolza::GetConstraintMatrixSparse(ocpSize_, x0, dynamicsArray, nullptr, nullptr, G_ref, g_ref);
  bolza::precondition::KktMatrixInPlace(5, H_ref, h_ref, G_ref, g_ref, D_ref, E_ref, c_ref);

  // Test start
  bolza::vector_array_t D_array, E_array;
  bolza::scalar_t c;
  bolza::vector_array_t scalingVectors(N_);
  bolza::precondition::OcpDataInPlaceInParallel(threadPool, x0, ocpSize_, 5, dynamicsArray, costArray, D_array, E_array, scalingVectors, c);

  bolza::vector_t D_stacked(D_ref.rows()), E_stacked(E_ref.rows());
  int curRow = 0;
  for (auto& v : D_array) {
    D_stacked.segment(curRow, v.size()) = v;
    curRow += v.size();
  }
  curRow = 0;
  for (auto& v : E_array) {
    E_stacked.segment(curRow, v.size()) = v;
    curRow += v.size();
  }

  EXPECT_TRUE(D_stacked.isApprox(D_ref));
  EXPECT_TRUE(E_stacked.isApprox(E_ref));
  EXPECT_DOUBLE_EQ(c, c_ref);

  Eigen::SparseMatrix<bolza::scalar_t> H_scaledData;
  bolza::vector_t h_scaledData;
  bolza::GetCostMatrixSparse(ocpSize_, x0, costArray, H_scaledData, h_scaledData);
  Eigen::SparseMatrix<bolza::scalar_t> G_scaledData;
  bolza::vector_t g_scaledData;
  bolza::GetConstraintMatrixSparse(ocpSize_, x0, dynamicsArray, nullptr, &scalingVectors, G_scaledData, g_scaledData);

  EXPECT_TRUE(H_ref.isApprox(H_scaledData));  // H
  EXPECT_TRUE(h_ref.isApprox(h_scaledData));  // h
  EXPECT_TRUE(G_ref.isApprox(G_scaledData));  // G
  EXPECT_TRUE(g_ref.isApprox(g_scaledData));  // g
}

TEST_F(PreconditionTest, descaleSolution) {
  bolza::vector_array_t D(2 * N_);
  bolza::vector_t dStacked(numDecisionVariables_);
  bolza::vector_array_t x(N_ + 1), u(N_);
  x[0].setRandom(nx_);
  for (int i = 0; i < N_; i++) {
    D[2 * i].setRandom(nu_);
    D[2 * i + 1].setRandom(nx_);
    u[i].setRandom(nu_);
    x[i + 1].setRandom(nx_);
  }
  int curRow = 0;
  for (auto& v : D) {
    dStacked.segment(curRow, v.size()) = v;
    curRow += v.size();
  }
  bolza::vector_t packedSolution;
  bolza::ToKktSolution(x, u, packedSolution);

  packedSolution.array() *= dStacked.array();

  bolza::vector_t packedSolutionNew;
  bolza::precondition::DescaleSolution(D, x, u);
  bolza::ToKktSolution(x, u, packedSolutionNew);
  EXPECT_TRUE(packedSolutionNew.isApprox(packedSolution)) << std::setprecision(6) << "DescaledSolution: \n"
                                                          << packedSolutionNew.transpose() << "\nIt should be \n"
                                                          << packedSolution.transpose();
}