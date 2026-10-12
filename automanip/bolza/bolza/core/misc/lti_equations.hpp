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
 * @file lti_equations.hpp
 * @brief LTI system class for linear defined as \f$ dx/dt = G_m*x(t) + G_v \f$.
 */


#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include <bolza/core/integration/ode_base.hpp>

namespace bolza {

/**
 * LTI system class for linear defined as \f$ dx/dt = G_m*x(t) + G_v \f$.
 * with \f$ x \in R^{DIM1 \times DIM2} \f$.
 *
 * @tparam DIM1: First dimension of the state space.
 * @tparam DIM2: Second dimension of the state space.
 * @tparam SCALAR: data type
 */
template <int DIM1, int DIM2 = 1, typename SCALAR = double>
class LTIEquations : public OdeBase<DIM1 * DIM2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { LTI_DIM_ = DIM1 * DIM2 };

  using state_t = Eigen::Matrix<SCALAR, DIM1, DIM2>;
  using vectorized_state_t = Eigen::Matrix<SCALAR, LTI_DIM_, 1>;
  using state_array_t = std::vector<state_t, Eigen::aligned_allocator<state_t>>;
  using vectorized_state_array_t = std::vector<vectorized_state_t, Eigen::aligned_allocator<vectorized_state_t>>;

  /**
   * @brief Constructs a LTIEquations.
   */
  LTIEquations() = default;

  /**
   * @brief Destructor.
   */
  ~LTIEquations() = default;

  /**
   * Transcribes the stacked vector into its matrix form.
   *
   * @param [in] vector: a single vector constructed by concatenating columns of matrix.
   * @param [out] matrix: the original matrix
   */
  void static Convert2Matrix(const Eigen::Matrix<SCALAR, LTI_DIM_, 1>& vector, Eigen::Matrix<SCALAR, DIM1, DIM2>& matrix) {
    /**
     * @brief Data.
     * @return Result value.
     */
    matrix = Eigen::Map<const Eigen::Matrix<SCALAR, DIM1, DIM2>>(vector.data());
  }

  /**
   * Transcribes the state matrix type to a stacked vector.
   *
   * @param [in] matrix: the original matrix
   * @param [out] vector: a single vector constructed by concatenating columns of matrix.
   */
  void static Convert2Vector(const Eigen::Matrix<SCALAR, DIM1, DIM2>& matrix, Eigen::Matrix<SCALAR, LTI_DIM_, 1>& vector) {
    /**
     * @brief Data.
     * @return Result value.
     */
    vector = Eigen::Map<const Eigen::Matrix<SCALAR, LTI_DIM_, 1>>(matrix.data());
  }

    /**
   * @brief /**.
   *
   * Set the LTI system coefficients, \f$ dx/dt = G_m*x(t) + G_v \f$.
   *
   * /
   * @param [in] GmPtr: Gm Ptr.
   * @param [in] GvPtr: Gv Ptr.
   */
  void SetData(const Eigen::Matrix<SCALAR, DIM1, DIM1>* GmPtr, const Eigen::Matrix<SCALAR, DIM1, DIM2>* GvPtr) {
    GmPtr_ = GmPtr;
    GvPtr_ = GvPtr;
  }

  /**
   * Computes the LTI system dynamics.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [out] dxdt: Current state time derivative
   */
  void ComputeFlowMap(const SCALAR& t, const vectorized_state_t& x, vectorized_state_t& dxdt) override {
    /**
     * @brief Dxdt Matrix.
     * @param [in] DIM1: DIM1.
     * @param [in] DIM2: DIM2.
     * @return Result value.
     */
    Eigen::Map<Eigen::Matrix<SCALAR, DIM1, DIM2>> dxdt_Matrix(dxdt.data(), DIM1, DIM2);

    /**
     * @brief Data.
     * @param [in] GvPtr_: Gv Ptr.
     * @return Result value.
     */
    dxdt_Matrix = (*GmPtr_) * Eigen::Map<const Eigen::Matrix<SCALAR, DIM1, DIM2>>(x.data(), DIM1, DIM2) + (*GvPtr_);
  }

 private:
  // members required in computeFlowMap
  const Eigen::Matrix<SCALAR, DIM1, DIM1>* GmPtr_;
  const Eigen::Matrix<SCALAR, DIM1, DIM2>* GvPtr_;
};

}  // namespace bolza
