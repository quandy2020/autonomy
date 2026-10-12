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
 * @file sensitivity_sequential_riccati_equations.hpp
 * @brief Sensitivity sequential Riccati equations class.
 */


#include <Eigen/Dense>

#include <array>
#include <limits>

#include <bolza/core/dimensions.hpp>
#include <bolza/core/integration/ode_base.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/core/misc/numerics.hpp>

namespace bolza {

/**
 * Sensitivity sequential Riccati equations class
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SensitivitySequentialRiccatiEquations final : public OdeBase<STATE_DIM*(STATE_DIM + 1) / 2 + STATE_DIM + 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum {
    /** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
    S_DIM_ = STATE_DIM * (STATE_DIM + 1) / 2 + STATE_DIM + 1
  };

  using BASE = OdeBase<S_DIM_>;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using state_input_matrix_array_t = typename DIMENSIONS::state_input_matrix_array_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;
  using dynamic_matrix_array_t = typename DIMENSIONS::dynamic_matrix_array_t;

  using s_vector_t = Eigen::Matrix<scalar_t, S_DIM_, 1>;
  using s_vector_array_t = std::vector<s_vector_t, Eigen::aligned_allocator<s_vector_t> >;

  /**
   * Default constructor.
   */
  SensitivitySequentialRiccatiEquations() = default;

  /**
   * Default destructor.
   */
  ~SensitivitySequentialRiccatiEquations() override = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  SensitivitySequentialRiccatiEquations<STATE_DIM, INPUT_DIM>* Clone() const {
    return new SensitivitySequentialRiccatiEquations<STATE_DIM, INPUT_DIM>(*this);
  }

      /**
/**
 * @brief .
 *
 * Transcribe symmetric matrix nabla_Sm, vector nabla_Sv and
 *
 * scalar nabla_s into a single vector.
 *
 * nabla_Sm, nabla_Sv and nabla_s.
 *
 * static void Convert2Vector(const state_matrix_t& nabla_Sm, const state_vector_t& nabla_Sv, const scalar_t& nabla_s, s_vector_t& allSs) {
 *
 * /* nabla_Sm is symmetric. Here, we only extract the upper triangular part and transcribe it in column-wise fashion into allSs*/
 *
 * size_t count = 0;  // count the total number of scalar entries covered
 *
 * size_t nRows = 0;
 *
 * for (size_t nCols = 0; nCols < STATE_DIM; nCols++) {
 *
 * nRows = nCols + 1;
 *
 * allSs.segment(count, nRows) << Eigen::Map<const dynamic_vector_t>(nabla_Sm.data() + nCols * STATE_DIM, nRows);
 *
 * count += nRows;
 *
 * }
 *
 * /* add data from nabla_Sv on top
 * @param [in] STATE_DIM: STATE DIM.
 * @param [in] nRows: N Rows.
 * @return Result value.
 */
igen::Map<const dynamic_vector_t>(nabla_Sm.data() + nCols * STATE_DIM, nRows);
      count += nRows;
    }

    /* add data from nabla_Sv on top*/
    allSs.template segment<STATE_DIM>((STATE_DIM * (STATE_DIM + 1)) / 2) << Eigen::Map<const dynamic_vector_t>(nabla_Sv.data(), STATE_DIM);

    /* add nabla_s as last element*/
    allSs.template tail<1>() << nabla_s;
  }

      /**
      /**
       * @brief .
       *
       * Transcribes the stacked vector allSs into a symmetric matrix, nabla_Sm,
       *
       * a vector, nabla_Sv and a single scalar, nabla_s.
       *
       * nabla_Sv and nabla_s.
       *
       * static void Convert2Matrix(const s_vector_t& allSs, state_matrix_t& nabla_Sm, state_vector_t& nabla_Sv, scalar_t& nabla_s) {
       *
       * /* Sm is symmetric. Here, we map the first entries from allSs onto the respective elements in the symmetric matrix*/
       *
       * size_t count = 0;
       *
       * size_t nCols = 0;
       *
       * for (size_t rows = 0; rows < STATE_DIM; rows++) {
       *
       * nCols = rows + 1;
       *
       * nabla_Sm.block(rows, 0, 1, nCols) << Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols).transpose();
       *
       * // "nCols-1" because diagonal elements have already been covered
       *
       * nabla_Sm.block(0, rows, nCols - 1, 1) << Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols - 1);
       *
       * count += nCols;
       *
       * }
       *
       * /* extract the vector Sv
       * @param [in] rows: Rows.
       * @return Result value.
       */
      nabla_Sm.block(0, rows, nCols - 1, 1) << Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols - 1);
      count += nCols;
    }

    /* extract the vector Sv*/
    nabla_Sv = Eigen::Map<const dynamic_vector_t>(allSs.data() + (STATE_DIM * (STATE_DIM + 1)) / 2, STATE_DIM);

    /* extract s as the last element */
    nabla_s = allSs.template tail<1>()(0);
  }

    /**
   * @brief.
   *
   * Sets data
   *
   * /
   * @param [in] learningRate: Learning Rate.
   * @param [in] SsTimePtr: Ss Time Ptr.
   * @param [in] SmPtr: Sm Ptr.
   * @param [in] SvPtr: Sv Ptr.
   * @param [in] timeStampPtr: Time Stamp Ptr.
   * @param [in] modelDataPtr: Model Data Ptr.
   * @param [in] RmInversePtr: Rm Inverse Ptr.
   * @param [in] nablaqPtr: Nablaq Ptr.
   * @param [in] nablaQvPtr: Nabla Qv Ptr.
   * @param [in] nablaRvPtr: Nabla Rv Ptr.
   */
  void SetData(const scalar_t& learningRate, const scalar_array_t* SsTimePtr, const state_matrix_array_t* SmPtr,
               const state_vector_array_t* SvPtr, const scalar_array_t* timeStampPtr, const ModelData::array_t* modelDataPtr,
               const dynamic_matrix_array_t* RmInversePtr, const scalar_array_t* nablaqPtr, const state_vector_array_t* nablaQvPtr,
               const input_vector_array_t* nablaRvPtr) {
    alpha_ = learningRate;

    SsTimePtr_ = SsTimePtr;
    SmPtr_ = SmPtr;
    SvPtr_ = SvPtr;

    timeStampPtr_ = timeStampPtr;
    modelDataPtr_ = modelDataPtr;
    RmInversePtr_ = RmInversePtr;
    nablaqPtr_ = nablaqPtr;
    nablaQvPtr_ = nablaQvPtr;
    nablaRvPtr_ = nablaRvPtr;
  }

  /**
   * Sets the multiplier of exogenous part of the equation. It is either zero
   * or plus-minus 1/(s_{i+1}-s_{i})
   *
   * @param [in] multiplier: the multiplier of exogenous part of the equation.
   */
  void SetMultiplier(const scalar_t& multiplier) { multiplier_ = multiplier; }

    /**
   * @brief.
   *
   * Computes derivative
   *
   * /
   * @param [in] z: Z.
   * @param [in] allSs: All Ss.
   * @param [in,out] derivatives: Derivatives.
   */
  void ComputeFlowMap(const scalar_t& z, const s_vector_t& allSs, s_vector_t& derivatives) override {
    // denormalized time
    const scalar_t t = -z;

    /**
     * @brief Constructs a Convert2Matrix.
     * @param [in] allSs: All Ss.
     * @param [in] nabla_Sm_: Nabla Sm.
     * @param [in] nabla_Sv_: Nabla Sv.
     * @param [in] nabla_s_: Nabla s.
     */
    Convert2Matrix(allSs, nabla_Sm_, nabla_Sv_, nabla_s_);

    /**
     * @brief Time Segment.
     * @param [in] t: T.
     * @param [in] SsTimePtr_: Ss Time Ptr.
     * @return Result value.
     */
    auto indexAlpha = LinearInterpolation::TimeSegment(t, SsTimePtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Sv_: Sv.
     * @param [in] SvPtr_: Sv Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, Sv_, SvPtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Sm_: Sm.
     * @param [in] SmPtr_: Sm Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, Sm_, SmPtr_);

    /**
     * @brief Time Segment.
     * @param [in] t: T.
     * @param [in] timeStampPtr_: Time Stamp Ptr.
     * @return Result value.
     */
    indexAlpha = LinearInterpolation::TimeSegment(t, timeStampPtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Am_: Am.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] dynamicsStateDerivative: Dynamics State Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, Am_, modelDataPtr_, ModelData::dynamicsStateDerivative);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Bm_: Bm.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] dynamicsInputDerivative: Dynamics Input Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, Bm_, modelDataPtr_, ModelData::dynamicsInputDerivative);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] q_: Q.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] Cost: Cost.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, q_, modelDataPtr_, ModelData::Cost);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Qv_: Qv.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] costStateDerivative: Cost State Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, Qv_, modelDataPtr_, ModelData::costStateDerivative);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Rv_: Rv.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] costInputDerivative: Cost Input Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, Rv_, modelDataPtr_, ModelData::costInputDerivative);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Qm_: Qm.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] costStateSecondDerivative: Cost State Second Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, Qm_, modelDataPtr_, ModelData::costStateSecondDerivative);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Rm_: Rm.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] costInputSecondDerivative: Cost Input Second Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, Rm_, modelDataPtr_, ModelData::costInputSecondDerivative);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Pm_: Pm.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] costInputStateDerivative: Cost Input State Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, Pm_, modelDataPtr_, ModelData::costInputStateDerivative);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] invRm_: Inv Rm.
     * @param [in] RmInversePtr_: Rm Inverse Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, invRm_, RmInversePtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] nabla_q_: Nabla q.
     * @param [in] nablaqPtr_: Nablaq Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, nabla_q_, nablaqPtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] nabla_Qv_: Nabla Qv.
     * @param [in] nablaQvPtr_: Nabla Qv Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, nabla_Qv_, nablaQvPtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] nabla_Rv_: Nabla Rv.
     * @param [in] nablaRvPtr_: Nabla Rv Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, nabla_Rv_, nablaRvPtr_);

    /**
     * @brief Transpose.
     * @param [in,out] Sv_: Sv.
     * @return Result value.
     */
    Lv_ = invRm_ * (Rv_ + Bm_.transpose() * Sv_);
    /**
     * @brief Transpose.
     * @param [in,out] Sm_: Sm.
     * @return Result value.
     */
    Lm_ = invRm_ * (Pm_ + Bm_.transpose() * Sm_);

    /**
     * @brief Transpose.
     * @param [in,out] nabla_Sv_: Nabla Sv.
     * @return Result value.
     */
    nabla_Lv_ = invRm_ * (nabla_Rv_ + Bm_.transpose() * nabla_Sv_);
    nabla_Lm_ = invRm_ * Bm_.transpose() * nabla_Sm_;

    // Riccati equations
    if (!numerics::almost_eq(multiplier_, 0.0)) {
      dSmdt_ = Qm_ + Am_.transpose() * Sm_ + Sm_.transpose() * Am_ - Lm_.transpose() * Rm_ * Lm_;
      /**
       * @brief Transpose.
       * @return Result value.
       */
      dSmdt_ = 0.5 * (dSmdt_ + dSmdt_.transpose()).eval();
      dSvdt_ = Qv_ + Am_.transpose() * Sv_ - Lm_.transpose() * Rm_ * Lv_;
      /**
       * @brief Dot.
       * @param [in,out] Lv_: Lv.
       * @return Result value.
       */
      dsdt_ = q_ - 0.5 * alpha_ * (2.0 - alpha_) * Lv_.dot(Rm_ * Lv_);

    } else {
      /**
       * @brief Set Zero.
       * @return Result value.
       */
      dSmdt_.setZero();
      /**
       * @brief Set Zero.
       * @return Result value.
       */
      dSvdt_.setZero();
      dsdt_ = 0.0;
    }

    // derivatives of Riccati equations
    nabla_dSmdt_ =
        Am_.transpose() * nabla_Sm_ + nabla_Sm_.transpose() * Am_ - nabla_Lm_.transpose() * Rm_ * Lm_ - Lm_.transpose() * Rm_ * nabla_Lm_;
    /**
     * @brief Transpose.
     * @return Result value.
     */
    nabla_dSmdt_ = 0.5 * (nabla_dSmdt_ + nabla_dSmdt_.transpose()).eval();
    nabla_dSvdt_ = nabla_Qv_ + Am_.transpose() * nabla_Sv_ - nabla_Lm_.transpose() * Rm_ * Lv_ - Lm_.transpose() * Rm_ * nabla_Lv_;
    /**
     * @brief Dot.
     * @return Result value.
     */
    nabla_dsdt_ = nabla_q_ - 0.5 * alpha_ * (2.0 - alpha_) * (nabla_Lv_.dot(Rm_ * Lv_) + Lv_.dot(Rm_ * nabla_Lv_));

    // switching time gradient for the equivalent system
    nabla_dSmdz_ = nabla_dSmdt_ + multiplier_ * dSmdt_;
    nabla_dSvdz_ = nabla_dSvdt_ + multiplier_ * dSvdt_;
    nabla_dsdz_ = nabla_dsdt_ + multiplier_ * dsdt_;

    /**
     * @brief Constructs a Convert2Vector.
     * @param [in] nabla_dSmdz_: Nabla d Smdz.
     * @param [in] nabla_dSvdz_: Nabla d Svdz.
     * @param [in] nabla_dsdz_: Nabla dsdz.
     * @param [in] derivatives: Derivatives.
     */
    Convert2Vector(nabla_dSmdz_, nabla_dSvdz_, nabla_dsdz_, derivatives);
  }

 private:
  scalar_t alpha_ = 0.0;
  scalar_t multiplier_ = 0.0;

  const scalar_array_t* SsTimePtr_;
  const state_matrix_array_t* SmPtr_;
  const state_vector_array_t* SvPtr_;
  const scalar_array_t* timeStampPtr_;
  const ModelData::array_t* modelDataPtr_;
  const dynamic_matrix_array_t* RmInversePtr_;
  const scalar_array_t* nablaqPtr_;
  const state_vector_array_t* nablaQvPtr_;
  const input_vector_array_t* nablaRvPtr_;

  state_matrix_t nabla_Sm_;
  state_vector_t nabla_Sv_;
  scalar_t nabla_s_;
  state_vector_t Sv_;
  state_matrix_t Sm_;
  dynamic_matrix_t Am_;
  dynamic_matrix_t Bm_;
  scalar_t q_;
  dynamic_vector_t Qv_;
  dynamic_vector_t Rv_;
  dynamic_matrix_t Qm_;
  dynamic_matrix_t Rm_;
  dynamic_matrix_t Pm_;
  dynamic_matrix_t invRm_;
  scalar_t nabla_q_;
  state_vector_t nabla_Qv_;
  input_vector_t nabla_Rv_;

  input_vector_t Lv_;
  input_state_matrix_t Lm_;
  input_vector_t nabla_Lv_;
  input_state_matrix_t nabla_Lm_;

  scalar_t dsdt_;
  state_vector_t dSvdt_;
  state_matrix_t dSmdt_;

  // normalized derivatives of Riccati equations
  scalar_t nabla_dsdz_;
  state_vector_t nabla_dSvdz_;
  state_matrix_t nabla_dSmdz_;

  // derivatives of Riccati equations
  scalar_t nabla_dsdt_;
  state_vector_t nabla_dSvdt_;
  state_matrix_t nabla_dSmdt_;
};

}  // namespace bolza
