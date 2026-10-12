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
 * @file bvp_sensitivity_error_equations.hpp
 * @brief BVP sensitivity error equations.
 */


#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <vector>

#include <bolza/core/dimensions.hpp>
#include <bolza/core/integration/ode_base.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/core/model_data/model_data_linear_interpolation.hpp>

namespace bolza {

/**
 * BVP sensitivity error equations.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class BvpSensitivityErrorEquations final : public OdeBase<STATE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = OdeBase<STATE_DIM>;

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
  using constraint1_vector_t = typename DIMENSIONS::constraint1_vector_t;
  using constraint1_vector_array_t = typename DIMENSIONS::constraint1_vector_array_t;
  using constraint1_state_matrix_t = typename DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_state_matrix_array_t = typename DIMENSIONS::constraint1_state_matrix_array_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;
  using dynamic_matrix_array_t = typename DIMENSIONS::dynamic_matrix_array_t;

  /**
   * Constructor.
   */
  BvpSensitivityErrorEquations() = default;

  /**
   * Default destructor.
   */
  ~BvpSensitivityErrorEquations() override = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  BvpSensitivityErrorEquations<STATE_DIM, INPUT_DIM>* Clone() const {
    return new BvpSensitivityErrorEquations<STATE_DIM, INPUT_DIM>(*this);
  }

    /**
   * @brief /**.
   *
   * Sets Data
   *
   * /
   * @param [in] timeStampPtr: Time Stamp Ptr.
   * @param [in] modelDataPtr: Model Data Ptr.
   * @param [in] projectedModelDataPtr: Projected Model Data Ptr.
   * @param [in] RmInversePtr: Rm Inverse Ptr.
   * @param [in] RinvCholPtr: Rinv Chol Ptr.
   * @param [in] EvDevProjectedPtr: Ev Dev Projected Ptr.
   * @param [in] SmTimeStampPtr: Sm Time Stamp Ptr.
   * @param [in] SmPtr: Sm Ptr.
   */
  void SetData(const scalar_array_t* timeStampPtr, const ModelData::array_t* modelDataPtr, const ModelData::array_t* projectedModelDataPtr,
               const dynamic_matrix_array_t* RmInversePtr, const dynamic_matrix_array_t* RinvCholPtr,
               const input_vector_array_t* EvDevProjectedPtr, const scalar_array_t* SmTimeStampPtr, const state_matrix_array_t* SmPtr) {
    timeStampPtr_ = timeStampPtr;
    modelDataPtr_ = modelDataPtr;
    projectedModelDataPtr_ = projectedModelDataPtr;
    RmInversePtr_ = RmInversePtr;
    RinvCholPtr_ = RinvCholPtr;
    EvDevProjectedPtr_ = EvDevProjectedPtr;
    SmTimeStampPtr_ = SmTimeStampPtr;
    SmPtr_ = SmPtr;
  }

    /**
   * @brief /**.
   *
   * Computes Derivative
   *
   * /
   * @param [in] z: Z.
   * @param [in] Mve: Mve.
   * @param [in,out] dMvedz: D Mvedz.
   */
  void ComputeFlowMap(const scalar_t& z, const state_vector_t& Mve, state_vector_t& dMvedz) override {
    // denormalized time
    const scalar_t t = -z;
    /**
     * @brief Time Segment.
     * @param [in] t: T.
     * @param [in] timeStampPtr_: Time Stamp Ptr.
     * @return Result value.
     */
    auto indexAlpha = LinearInterpolation::TimeSegment(t, timeStampPtr_);

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
     * @param [in] AmConstrained_: Am Constrained.
     * @param [in] projectedModelDataPtr_: Projected Model Data Ptr.
     * @param [in] dynamicsStateDerivative: Dynamics State Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, AmConstrained_, projectedModelDataPtr_, ModelData::dynamicsStateDerivative);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] CmProjected_: Cm Projected.
     * @param [in] projectedModelDataPtr_: Projected Model Data Ptr.
     * @param [in] stateInputEqConstrStateDerivative: State Input Eq Constr State Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, CmProjected_, projectedModelDataPtr_, ModelData::stateInputEqConstrStateDerivative);

    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] RmInverse_: Rm Inverse.
     * @param [in] RmInversePtr_: Rm Inverse Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, RmInverse_, RmInversePtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] RinvChol_: Rinv Chol.
     * @param [in] RinvCholPtr_: Rinv Chol Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, RinvChol_, RinvCholPtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] EvDevProjected_: Ev Dev Projected.
     * @param [in] EvDevProjectedPtr_: Ev Dev Projected Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, EvDevProjected_, EvDevProjectedPtr_);

    /**
     * @brief Interpolate.
     * @param [in] t: T.
     * @param [in] Sm_: Sm.
     * @param [in] SmTimeStampPtr_: Sm Time Stamp Ptr.
     * @param [in] SmPtr_: Sm Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(t, Sm_, SmTimeStampPtr_, SmPtr_);

    // Lm
    // TODO: Double check if equations are correct after change to cholesky decomposition approach
    /**
     * @brief Transpose.
     * @param [in,out] Sm_: Sm.
     * @return Result value.
     */
    Lm_ = RinvChol_.transpose() * (Pm_ + Bm_.transpose() * Sm_);

    dMvedz =
        (AmConstrained_ - Bm_ * RinvChol_ * Lm_).transpose() * Mve + (CmProjected_ - RinvChol_ * Lm_).transpose() * Rm_ * EvDevProjected_;
  }

 private:
  scalar_t multiplier_ = 0.0;

  const scalar_array_t* timeStampPtr_;
  const ModelData::array_t* modelDataPtr_;
  const ModelData::array_t* projectedModelDataPtr_;
  const dynamic_matrix_array_t* RmInversePtr_;
  const dynamic_matrix_array_t* RinvCholPtr_;
  const input_vector_array_t* EvDevProjectedPtr_;
  const scalar_array_t* SmTimeStampPtr_;
  const state_matrix_array_t* SmPtr_;

  dynamic_matrix_t Bm_;
  dynamic_matrix_t Rm_;
  dynamic_matrix_t Pm_;
  dynamic_matrix_t AmConstrained_;
  dynamic_matrix_t CmProjected_;
  dynamic_matrix_t RmInverse_;
  dynamic_matrix_t RinvChol_;
  input_vector_t EvDevProjected_;
  state_matrix_t Sm_;
  dynamic_matrix_t Lm_;
};

}  // namespace bolza
