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

#pragma once

/**
 * @file linear_algebra.hpp
 * @brief Set the eigenvalues of a triangular matrix to a minimum magnitude (maintaining the sign).
 */


#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

#include <bolza/core/numeric_traits.hpp>
#include <bolza/core/types.hpp>

namespace bolza {
namespace LinearAlgebra {

/**
 * @brief.
 *
 * Set the eigenvalues of a triangular matrix to a minimum magnitude (maintaining the sign).
 *
 * /
 * @param [in,out] Lr: Lr.
 * @param [in] minEigenValue: Min Eigen Value.
 */
void SetTriangularMinimumEigenvalues(matrix_t& Lr, scalar_t minEigenValue = numeric_traits::WeakEpsilon<scalar_t>());

/**
 * Makes the input matrix PSD using a eigenvalue decomposition.
 *
 * @param [in,out] squareMatrix: The matrix to become PSD.
 * @param [in] minEigenvalue: minimum eigenvalue.
 */
void MakePsdEigenvalue(matrix_t& squareMatrix, scalar_t minEigenvalue = numeric_traits::LimitEpsilon<scalar_t>());

/**
 * Makes the input matrix PSD based on Gershgorin circle theorem. If the input matrix is positive definite and diagonally dominant,
 * the method will not modify the matrix.
 *
 * How it works:
 * Assume that the Ri is the sum of the absolute values of the non-diagonal entries in the i-th row (Gershgorin radius).
 * This methods updates the diagonal elements as Aii = max(Aii, minEigenvalue + Ri).
 *
 * To understand this update rule note that the Gershgorin radius (Ri) of the new matrix is the same as original one.
 * Now if we use the Gershgorin circle theorem we have:
 * | lambda - max(Aii, minEigenvalue + Ri) | < Ri ==> max(Aii, minEigenvalue + Ri) - Ri < lambda < max(Aii, minEigenvalue + Ri) + Ri
 * Two cases are possible:
 * (1) Aii < minEigenvalue + Ri ==> minEigenvalue < lambda < minEigenvalue + 2 Ri
 * (2) Aii > minEigenvalue + Ri ==> minEigenvalue < Aii - Ri < lambda < Aii + Ri
 *
 * @param [in,out] squareMatrix: The matrix to become PSD.
 * @param [in] minEigenvalue: minimum eigenvalue.
 */
void MakePsdGershgorin(matrix_t& squareMatrix, scalar_t minEigenvalue = numeric_traits::LimitEpsilon<scalar_t>());

/**
 * Makes the input matrix PSD based on modified Cholesky decomposition.
 *
 * S P' (A + E) P S = L L'
 * where P is a permutation matrix, E is a diagonal perturbation matrix, L is unit lower triangular, and D is diagonal.
 * If A is sufficiently positive definite, then the perturbation matrix E will be zero and this method is equivalent to
 * the pivoted Cholesky algorithm. For indefinite matrices, the perturbation matrix E is computed to ensure that A + E
 * is positive definite and well conditioned.
 *
 * References : C-J. Lin and J. J. Moré, Incomplete Cholesky Factorizations with Limited memory, SIAM J. Sci. Comput.
 * 21(1), pp. 24-45, 1999
 *
 * @param [in,out] A: The matrix to become PSD.
 * @param [in] minEigenvalue: minimum eigenvalue.
 */
void MakePsdCholesky(matrix_t& A, scalar_t minEigenvalue = numeric_traits::LimitEpsilon<scalar_t>());

/**
 * Computes the U*U^T decomposition associated to the inverse of the input matrix, where U is an upper triangular
 * matrix. Note that the U*U^T decomposition is different from the Cholesky decomposition (U^T*U).
 *
 * @param [in] Am: A symmetric square positive definite matrix
 * @param [out] AmInvUmUmT: The upper-triangular matrix associated to the UUT decomposition of inv(Am) matrix.
 */
void ComputeInverseMatrixUUT(const matrix_t& Am, matrix_t& AmInvUmUmT);

/**
 * Computes constraint projection for linear constraints  C*x + D*u - e = 0, with the weighting inv(Rm)
 * The input cost matrix Rm should be already inverted and decomposed such that inv(Rm) = RmInvUmUmT * RmInvUmUmT^T.
 *
 * @param [in] Dm: A full row rank constraint matrix
 * @param [in] RmInvUmUmT: The upper-triangular matrix associated to the UUT decomposition of inv(Rm) matrix.
 * @param [out] DmDagger: The right weighted pseudo inverse of Dm, DmDagger = inv(Rm)*Dm'*inv(Dm*inv(Rm)*Dm')
 * @param [out] DmDaggerTRmDmDaggerUUT: The UUT decomposition of DmDagger^T * Rm * DmDagger = Um * Um^T
 * @param [out] RmInvConstrainedUUT: The VVT decomposition of (I-DmDagger*Dm) * inv(Rm) * (I-DmDagger*Dm)^T where V is of
 * the dimension n_u*(n_u-n_c) with n_u = Rm.rows() and n_c = Dm.rows().
 */
void ComputeConstraintProjection(const matrix_t& Dm, const matrix_t& RmInvUmUmT, matrix_t& DmDagger, matrix_t& DmDaggerTRmDmDaggerUUT,
                                 matrix_t& RmInvConstrainedUUT);

/**
 * Returns the linear projection
 *  u = Pu * \tilde{u} + Px * x + Pe
 *
 * s.t. C*x + D*u + e = 0 is satisfied for any \tilde{u}
 *
 * Implementation based on the QR decomposition
 *
 * @param [in] constraint: C = dfdx, D = dfdu, e = f;
 * @return Projection terms Px = dfdx, Pu = dfdu, Pe = f (first) and left pseudo-inverse of D^T (second);
 */
std::pair<VectorFunctionLinearApproximation, matrix_t> QrConstraintProjection(const VectorFunctionLinearApproximation& constraint);

/**
 * Returns the linear projection
 *  u = Pu * \tilde{u} + Px * x + Pe
 *
 * s.t. C*x + D*u + e = 0 is satisfied for any \tilde{u}
 *
 * Implementation based on the LU decomposition
 *
 * @param [in] constraint: C = dfdx, D = dfdu, e = f;
 * @param [in] extractPseudoInverse: If true, left pseudo-inverse of D^T is returned. If false, an empty matrix is returned;
 * @return Projection terms Px = dfdx, Pu = dfdu, Pe = f (first) and left pseudo-inverse of D^T (second);
 */
std::pair<VectorFunctionLinearApproximation, matrix_t> LuConstraintProjection(const VectorFunctionLinearApproximation& constraint,
                                                                              bool extractPseudoInverse = false);

/** Computes the rank of a matrix */
template <typename Derived>
/**
 * @brief Rank.
 * @param [in] A: A.
 * @return Result value.
 */
int Rank(const Derived& A) {
  /**
   * @brief Col Piv Householder Qr.
   * @return Result value.
   */
  return A.colPivHouseholderQr().rank();
}

/** Computes the complex eigenvalues of A */
template <typename Derived>
/**
 * @brief Eigenvalues.
 * @param [in] A: A.
 * @return Result value.
 */
Eigen::VectorXcd eigenvalues(const Derived& A) {
  /**
   * @brief Eigenvalues.
   * @return Result value.
   */
  return A.eigenvalues();
}

/** Computes the eigenvalues for a symmetric matrix A */
template <typename Derived>
/**
 * @brief Symmetric Eigenvalues.
 * @param [in] A: A.
 * @return Result value.
 */
vector_t SymmetricEigenvalues(const Derived& A) {
  /**
   * @brief Eigenvalues.
   * @return Result value.
   */
  return A.template selfadjointView<Eigen::Lower>().eigenvalues();
}

// Declaring explicit instantiations for dynamic sized matrices
/**
 * @brief Rank.
 * @param [in] A: A.
 * @return Result value.
 */
extern template int Rank(const matrix_t& A);
/**
 * @brief Eigenvalues.
 * @param [in] A: A.
 * @return Result value.
 */
extern template Eigen::VectorXcd eigenvalues(const matrix_t& A);
/**
 * @brief Symmetric Eigenvalues.
 * @param [in] A: A.
 * @return Result value.
 */
extern template vector_t SymmetricEigenvalues(const matrix_t& A);

}  // namespace LinearAlgebra
}  // namespace bolza
