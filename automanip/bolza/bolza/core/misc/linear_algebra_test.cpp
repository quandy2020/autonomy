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

#include <bolza/core/types.hpp>
#include <bolza/core/misc/linear_algebra.hpp>
#include <bolza/core/misc/random_matrices.hpp>

TEST(test_projection, testProjectionQR) {
  constexpr bolza::scalar_t nx = 30;
  constexpr bolza::scalar_t nu = 20;
  constexpr bolza::scalar_t nc = 10;
  const auto constraint = [&]() {
    bolza::VectorFunctionLinearApproximation approx;
    approx.dfdx = bolza::matrix_t::Random(nc, nx);
    approx.dfdu = bolza::matrix_t::Random(nc, nu);
    approx.f = bolza::vector_t::Random(nc);
    return approx;
  }();

  auto result = bolza::LinearAlgebra::QrConstraintProjection(constraint);
  const auto projection = std::move(result.first);
  const auto pseudoInverse = std::move(result.second);

  // range of Pu is in null-space of D
  ASSERT_TRUE((constraint.dfdu * projection.dfdu).isZero());

  // D * Px cancels the C term
  ASSERT_TRUE((constraint.dfdx + constraint.dfdu * projection.dfdx).isZero());

  // D * Pe cancels the e term
  ASSERT_TRUE((constraint.f + constraint.dfdu * projection.f).isZero());

  ASSERT_EQ(pseudoInverse.rows(), constraint.dfdu.rows());
  ASSERT_EQ(pseudoInverse.cols(), constraint.dfdu.cols());

  ASSERT_TRUE((pseudoInverse * constraint.dfdu.transpose()).isIdentity());
  ASSERT_TRUE((pseudoInverse.transpose() * constraint.dfdx).isApprox(-projection.dfdx));
  ASSERT_TRUE((pseudoInverse.transpose() * constraint.f).isApprox(-projection.f));
}

TEST(test_projection, testProjectionLU) {
  constexpr bolza::scalar_t nx = 30;
  constexpr bolza::scalar_t nu = 20;
  constexpr bolza::scalar_t nc = 10;
  const auto constraint = [&]() {
    bolza::VectorFunctionLinearApproximation approx;
    approx.dfdx = bolza::matrix_t::Random(nc, nx);
    approx.dfdu = bolza::matrix_t::Random(nc, nu);
    approx.f = bolza::vector_t::Random(nc);
    return approx;
  }();

  auto result = bolza::LinearAlgebra::LuConstraintProjection(constraint);
  ASSERT_EQ(result.second.rows(), 0);
  ASSERT_EQ(result.second.cols(), 0);

  const auto projection = std::move(result.first);

  // range of Pu is in null-space of D
  ASSERT_TRUE((constraint.dfdu * projection.dfdu).isZero());

  // D * Px cancels the C term
  ASSERT_TRUE((constraint.dfdx + constraint.dfdu * projection.dfdx).isZero());

  // D * Pe cancels the e term
  ASSERT_TRUE((constraint.f + constraint.dfdu * projection.f).isZero());

  auto resultWithPseudoInverse = bolza::LinearAlgebra::LuConstraintProjection(constraint, true);
  const auto projectionWithPseudoInverse = std::move(resultWithPseudoInverse.first);
  ASSERT_TRUE(projection.f.isApprox(projectionWithPseudoInverse.f));
  ASSERT_TRUE(projection.dfdx.isApprox(projectionWithPseudoInverse.dfdx));
  ASSERT_TRUE(projection.dfdu.isApprox(projectionWithPseudoInverse.dfdu));

  const auto pseudoInverse = std::move(resultWithPseudoInverse.second);
  ASSERT_EQ(pseudoInverse.rows(), constraint.dfdu.rows());
  ASSERT_EQ(pseudoInverse.cols(), constraint.dfdu.cols());

  ASSERT_TRUE((pseudoInverse * constraint.dfdu.transpose()).isIdentity());
  ASSERT_TRUE((pseudoInverse.transpose() * constraint.dfdx).isApprox(-projection.dfdx));
  ASSERT_TRUE((pseudoInverse.transpose() * constraint.f).isApprox(-projection.f));
}

TEST(LLTofInverse, checkAgainstFullInverse) {
  constexpr size_t n = 10;        // matrix size
  constexpr bolza::scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // Some random symmetric positive definite matrix
  bolza::matrix_t A = bolza::LinearAlgebra::GenerateSPDmatrix<bolza::matrix_t>(n);
  bolza::matrix_t AmInvUmUmT;

  bolza::LinearAlgebra::ComputeInverseMatrixUUT(A, AmInvUmUmT);

  bolza::matrix_t Ainv = A.inverse();
  bolza::matrix_t Ainv_constructed = AmInvUmUmT * AmInvUmUmT.transpose();

  ASSERT_LT((Ainv - Ainv_constructed).array().abs().maxCoeff(), tol);
}

TEST(constraintProjection, checkAgainstFullComputations) {
  constexpr size_t m = 4;         // num constraints
  constexpr size_t n = 15;        // num inputs
  constexpr bolza::scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // Some random constraint matrix
  bolza::matrix_t D = bolza::LinearAlgebra::GenerateFullRowRankmatrix(m, n);

  // Random cost matrix
  bolza::matrix_t R = bolza::LinearAlgebra::GenerateSPDmatrix<bolza::matrix_t>(n);

  // Inverse of R
  bolza::matrix_t RmInvUmUmT;
  bolza::LinearAlgebra::ComputeInverseMatrixUUT(R, RmInvUmUmT);
  bolza::matrix_t Rinv = RmInvUmUmT * RmInvUmUmT.transpose();

  // Compute constraint projection terms, this is what we are testing in this unit test
  bolza::matrix_t Ddagger, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol;
  bolza::LinearAlgebra::ComputeConstraintProjection(D, RmInvUmUmT, Ddagger, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol);

  // Reconstruct full matrices to compare
  bolza::matrix_t RinvConstrained = RinvConstrainedChol * RinvConstrainedChol.transpose();
  bolza::matrix_t DdaggerT_R_Ddagger = DdaggerT_R_Ddagger_Chol * DdaggerT_R_Ddagger_Chol.transpose();

  // Alternative computation following Farshidian - An Efficient Optimal Planning and Control Framework For Quadrupedal Locomotion
  // Check Ddagger
  bolza::matrix_t RmProjected = (D * Rinv * D.transpose()).ldlt().Solve(bolza::matrix_t::Identity(m, m));
  bolza::matrix_t Ddagger_check = Rinv * D.transpose() * RmProjected;
  ASSERT_LT((Ddagger - Ddagger_check).array().abs().maxCoeff(), tol);

  // Check Ddagger^T * R * Ddagger
  bolza::matrix_t DdaggerT_R_Ddagger_check = Ddagger_check.transpose() * R * Ddagger_check;
  ASSERT_LT((DdaggerT_R_Ddagger - DdaggerT_R_Ddagger_check).array().abs().maxCoeff(), tol);

  // Check Constrained cost matrix defined as defined in the paper
  bolza::matrix_t nullspaceProjection = bolza::matrix_t::Identity(n, n) - Ddagger_check * D;
  bolza::matrix_t RinvConstrained_check = Rinv.transpose() * nullspaceProjection.transpose() * R * nullspaceProjection * Rinv;
  ASSERT_LT((RinvConstrained - RinvConstrained_check).array().abs().maxCoeff(), tol);
}

TEST(makePsdGershgorin, makePsdGershgorin) {
  constexpr size_t n = 10;        // matrix size
  constexpr bolza::scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // a random, symmetric, and diagonally dominant matrix
  bolza::matrix_t ddMat = bolza::LinearAlgebra::GenerateSPDmatrix<bolza::matrix_t>(n);
  bolza::matrix_t ddMatCorr = ddMat;
  bolza::LinearAlgebra::MakePsdGershgorin(ddMatCorr);
  ASSERT_TRUE(ddMat.isApprox(ddMatCorr, tol));

  // non-definite matrix
  auto lambdaMin = bolza::LinearAlgebra::SymmetricEigenvalues(ddMat).minCoeff();
  bolza::matrix_t ndMat = ddMat - (lambdaMin + 1e-2) * bolza::matrix_t::Identity(n, n);
  bolza::matrix_t ndMatCorr = ndMat;
  const bolza::scalar_t minDesiredEigenvalue = 1e-3;
  bolza::LinearAlgebra::MakePsdGershgorin(ndMatCorr, minDesiredEigenvalue);
  bolza::vector_t lambda = bolza::LinearAlgebra::SymmetricEigenvalues(ndMat);
  bolza::vector_t lambdaCorr = bolza::LinearAlgebra::SymmetricEigenvalues(ndMatCorr);
  std::cerr << "MakePSD Gershgorin:" << std::endl;
  std::cerr << "eigenvalues            " << lambda.transpose() << std::endl;
  std::cerr << "eigenvalues corrected: " << lambdaCorr.transpose() << std::endl;
  ASSERT_GE(lambdaCorr.minCoeff(), minDesiredEigenvalue);
}

TEST(makePsdCholesky, makePsdCholesky) {
  constexpr size_t n = 10;        // matrix size
  constexpr bolza::scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // PSD matrix check
  // some random symmetric matrix
  bolza::matrix_t psdMat = bolza::LinearAlgebra::GenerateSPDmatrix<bolza::matrix_t>(n);
  bolza::matrix_t psdMatCorr = psdMat;
  bolza::LinearAlgebra::MakePsdCholesky(psdMatCorr, 0.0);
  ASSERT_TRUE(psdMat.isApprox(psdMatCorr, tol));

  // non-definite matrix
  auto lambdaMin = bolza::LinearAlgebra::SymmetricEigenvalues(psdMat).minCoeff();
  bolza::matrix_t ndMat = psdMat - (lambdaMin + 1e-2) * bolza::matrix_t::Identity(n, n);
  bolza::matrix_t ndMatCorr = ndMat;
  constexpr bolza::scalar_t minDesiredEigenvalue = 1e-1;
  bolza::LinearAlgebra::MakePsdCholesky(ndMatCorr, minDesiredEigenvalue);
  bolza::vector_t lambda = bolza::LinearAlgebra::SymmetricEigenvalues(ndMat);
  bolza::vector_t lambdaCorr = bolza::LinearAlgebra::SymmetricEigenvalues(ndMatCorr);
  std::cerr << "MakePSD Cholesky: " << std::endl;
  std::cerr << "eigenvalues            " << lambda.transpose() << std::endl;
  std::cerr << "eigenvalues corrected: " << lambdaCorr.transpose() << std::endl;
  ASSERT_GE(lambdaCorr.minCoeff(), minDesiredEigenvalue);

  // zero matrix
  bolza::matrix_t zeroMat = bolza::matrix_t::Zero(n, n);
  bolza::LinearAlgebra::MakePsdCholesky(zeroMat, minDesiredEigenvalue);
  bolza::vector_t lambdaZeroMat = bolza::LinearAlgebra::SymmetricEigenvalues(zeroMat);
  ASSERT_GE(lambdaZeroMat.minCoeff(), minDesiredEigenvalue);

  // sparse matrix
  Eigen::VectorXd vec = Eigen::VectorXd::Random(n);
  vec = vec.unaryExpr([](bolza::scalar_t x) { return std::max(x, 0.0); });
  std::cerr << "Sparse vec:\n" << vec << std::endl;
  bolza::matrix_t sparseMat = vec * vec.transpose() - minDesiredEigenvalue * bolza::matrix_t::Identity(n, n);
  bolza::vector_t lambdaSparseMat = bolza::LinearAlgebra::SymmetricEigenvalues(sparseMat);
  bolza::LinearAlgebra::MakePsdCholesky(sparseMat, minDesiredEigenvalue);
  bolza::vector_t lambdaSparseMatCorr = bolza::LinearAlgebra::SymmetricEigenvalues(sparseMat);

  std::cerr << "MakePSD Cholesky Sparse Matrix: " << std::endl;
  std::cerr << "Sparse Matrix:\n" << sparseMat << std::endl;
  std::cerr << "Sparse Matrix eigenvalues            " << lambdaSparseMat.transpose() << std::endl;
  std::cerr << "Sparse Matrix eigenvalues corrected: " << lambdaSparseMatCorr.transpose() << std::endl;

  ASSERT_GE(lambdaSparseMatCorr.minCoeff(), minDesiredEigenvalue);
}
