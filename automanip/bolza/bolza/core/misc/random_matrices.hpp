#pragma once

/**
 * @file random_matrices.hpp
 * @brief Compute random, symmetric, positive definite (SPD), and diagonally dominant matrix.
 */


#include <bolza/core/types.hpp>

namespace bolza {
namespace LinearAlgebra {
/**
 * Compute random, symmetric, positive definite (SPD), and diagonally dominant matrix.
 * @return random SPD matrix.
 */
template <typename MatrixType>
/**
 * @brief Generate SPDmatrix.
 * @return Result value.
 */
MatrixType GenerateSPDmatrix() {
  MatrixType A;
  /**
   * @brief Set Random.
   * @return Result value.
   */
  A.setRandom();
  /**
   * @brief Transpose.
   * @return Result value.
   */
  A = 0.5 * (A + A.transpose()).eval();    // avoid aliasing
  A.diagonal().array() += A.rows() * 1.0;  // makes the matrix diagonally dominant
  return A;
}

/**
 * Compute random, symmetric, positive definite (SPD), and diagonally dominant matrix of dimension size.
 * @param [in] size: Matrix dimension.
 * @return random SPD matrix.
 */
template <typename MatrixType>
/**
 * @brief Generate SPDmatrix.
 * @param [in] size: Size.
 * @return Result value.
 */
MatrixType GenerateSPDmatrix(int size) {
  /**
   * @brief A.
   * @param [in] size: Size.
   * @param [in] size: Size.
   * @return Result value.
   */
  MatrixType A(size, size);
  /**
   * @brief Set Random.
   * @return Result value.
   */
  A.setRandom();
  /**
   * @brief Transpose.
   * @return Result value.
   */
  A = 0.5 * (A + A.transpose()).eval();    // Avoid aliasing
  A.diagonal().array() += A.rows() * 1.0;  // makes the matrix diagonally dominant
  return A;
}

/**
 * @brief /**.
 *
 * Compute random full row rank matrix
 *
 * /
 * @param [in] m: M.
 * @param [in] n: N.
 * @return random SPD matrix.
 */
matrix_t GenerateFullRowRankmatrix(size_t m, size_t n) {
  if (m > n) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("[generateFullRowRankmatrix] Can't generate matrix with more rows than columns");
  };

  // Some random constraint matrix
  /**
   * @brief Random.
   * @param [in] m: M.
   * @param [in] n: N.
   * @return Result value.
   */
  matrix_t A = matrix_t::Random(m, n);
  /**
   * @brief Block.
   * @param [in] m: M.
   * @return Result value.
   */
  A.block(0, 0, m, m).setIdentity();  // Makes sure rows are independent
  return A;
}
}  // namespace LinearAlgebra
}  // namespace bolza
