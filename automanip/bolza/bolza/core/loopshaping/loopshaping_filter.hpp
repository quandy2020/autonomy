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
 * @file loopshaping_filter.hpp
 * @brief loopshaping filter (bolza).
 */


#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/transfer_function_base.hpp>
#include <Eigen/Dense>

namespace bolza {

class Filter {
 public:
  using diag_matrix_t = Eigen::DiagonalMatrix<bolza::scalar_t, Eigen::Dynamic>;

  /**
   * @brief Constructs a Filter.
   */
  Filter();

  /**
   * @brief Constructs a Filter.
   * @param [in] A: A.
   * @param [in] B: B.
   * @param [in] C: C.
   * @param [in] D: D.
   */
  Filter(matrix_t A, matrix_t B, matrix_t C, matrix_t D);

  /**
   * @brief Get Num States.
   * @return Result value.
   */
  size_t GetNumStates() const { return numStates_; };
  /**
   * @brief Get Num Inputs.
   * @return Result value.
   */
  size_t GetNumInputs() const { return numInputs_; };
  /**
   * @brief Get Num Outputs.
   * @return Result value.
   */
  size_t GetNumOutputs() const { return numOutputs_; };

  /**
   * @brief Get A.
   * @return Result value.
   */
  const matrix_t& GetA() const { return A_; }
  /**
   * @brief Get B.
   * @return Result value.
   */
  const matrix_t& GetB() const { return B_; }
  /**
   * @brief Get C.
   * @return Result value.
   */
  const matrix_t& GetC() const { return C_; }
  /**
   * @brief Get D.
   * @return Result value.
   */
  const matrix_t& GetD() const { return D_; }

  /// Get the diagonal of the filter matrices
  /**
   * @brief Get Adiag.
   * @return Result value.
   */
  const diag_matrix_t& GetAdiag() const { return a_; }
  /**
   * @brief Get Bdiag.
   * @return Result value.
   */
  const diag_matrix_t& GetBdiag() const { return b_; }
  /**
   * @brief Get Cdiag.
   * @return Result value.
   */
  const diag_matrix_t& GetCdiag() const { return c_; }
  /**
   * @brief Get Ddiag.
   * @return Result value.
   */
  const diag_matrix_t& GetDdiag() const { return d_; }

  /// Get the equivalent element-wise scaling for pre- and post multiplying with diagonal matrices.
  /**
   * @brief Get Scaling Cdiag Cdiag.
   * @return Result value.
   */
  const matrix_t& GetScalingCdiagCdiag() const { return diagCC_; }
  /**
   * @brief Get Scaling Ddiag Cdiag.
   * @return Result value.
   */
  const matrix_t& GetScalingDdiagCdiag() const { return diagDC_; }
  /**
   * @brief Get Scaling Ddiag Ddiag.
   * @return Result value.
   */
  const matrix_t& GetScalingDdiagDdiag() const { return diagDD_; }

  /**
   * @brief Print.
   */
  void Print() const;

  /**
   * @brief Find Equilibrium For Output.
   * @param [in] y: Y.
   * @param [in,out] x: X.
   * @param [in,out] u: U.
   */
  void FindEquilibriumForOutput(const vector_t& y, vector_t& x, vector_t& u) const;
  /**
   * @brief Find Equilibrium For Output Given State.
   * @param [in] y: Y.
   * @param [in] x: X.
   * @param [in,out] u: U.
   */
  void FindEquilibriumForOutputGivenState(const vector_t& y, const vector_t& x, vector_t& u) const;
  /**
   * @brief Find Equilibrium For Input.
   * @param [in] u: U.
   * @param [in,out] x: X.
   * @param [in,out] y: Y.
   */
  void FindEquilibriumForInput(const vector_t& u, vector_t& x, vector_t& y) const;

 private:
  /**
   * @brief Check Size.
   */
  void CheckSize() const;

  matrix_t A_, B_, C_, D_;
  diag_matrix_t a_, b_, c_, d_;
  matrix_t diagCC_, diagDC_, diagDD_;
  size_t numStates_ = 0;
  size_t numInputs_ = 0;
  size_t numOutputs_ = 0;
  Eigen::ColPivHouseholderQR<matrix_t> Aqr_;
  Eigen::ColPivHouseholderQR<matrix_t> Dqr_;
  Eigen::ColPivHouseholderQR<matrix_t> ABCDqr_;
};

}  // namespace bolza
