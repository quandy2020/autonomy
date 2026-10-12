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
 * @file bvp_sensitivity_equations.hpp
 * @brief BVP sensitivity equations.
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
 * BVP sensitivity equations.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class BvpSensitivityEquations final : public OdeBase<STATE_DIM> {
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
  using dynamic_vector_array_t = typename DIMENSIONS::dynamic_vector_array_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;

  /**
   * Constructor.
   */
  BvpSensitivityEquations() = default;

  /**
   * Default destructor.
   */
  ~BvpSensitivityEquations() override = default;

  /**
   * @brief Returns pointer to the class.
   * @param [in] this: This.
   * @return A raw pointer to the class.
   */
  BvpSensitivityEquations<STATE_DIM, INPUT_DIM>* Clone() const { return new BvpSensitivityEquations<STATE_DIM, INPUT_DIM>(*this); }

    /**
   * @brief /**.
   *
   * Sets Data
   *
   * /
   * @param [in] timeStampPtr: Time Stamp Ptr.
   * @param [in] modelDataPtr: Model Data Ptr.
   * @param [in] projectedModelDataPtr: Projected Model Data Ptr.
   * @param [in] costatePtr: Costate Ptr.
   * @param [in] lagrangianPtr: Lagrangian Ptr.
   * @param [in] controllerTimeStampPtr: Controller Time Stamp Ptr.
   * @param [in] KmConstrainedPtr: Km Constrained Ptr.
   * @param [in] SmPtr: Sm Ptr.
   */
  void SetData(const scalar_array_t* timeStampPtr, const ModelData::array_t* modelDataPtr, const ModelData::array_t* projectedModelDataPtr,
               const state_vector_array_t* costatePtr, const dynamic_vector_array_t* lagrangianPtr,
               const scalar_array_t* controllerTimeStampPtr, const input_state_matrix_array_t* KmConstrainedPtr,
               const state_matrix_array_t* SmPtr) {
    timeStampPtr_ = timeStampPtr;
    modelDataPtr_ = modelDataPtr;
    projectedModelDataPtr_ = projectedModelDataPtr;
    costatePtr_ = costatePtr;
    lagrangianPtr_ = lagrangianPtr;
    controllerTimeStampPtr_ = controllerTimeStampPtr;
    KmConstrainedPtr_ = KmConstrainedPtr;
    SmPtr_ = SmPtr;
  }

  /**
   * Sets the multiplier of exogenous part of the equation. It is either zero
   * or plus-minus 1/(s_{i+1}-s_{i})
   *
   * @param [in] multiplier: the multiplier of exogenous part of the equation.
   */
  void SetMultiplier(const scalar_t& multiplier) { multiplier_ = multiplier; }

    /**
   * @brief /**.
   *
   * Computes Derivative
   *
   * /
   * @param [in] z: Z.
   * @param [in] Mv: transition state
   * @param [in,out] dMvdz: D Mvdz.
   */
  void ComputeFlowMap(const scalar_t& z, const state_vector_t& Mv, state_vector_t& dMvdz) override {
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
     * @param [in] Fv_: Fv.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] dynamics: Dynamics.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, Fv_, modelDataPtr_, ModelData::dynamics);
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
     * @param [in] Qv_: Qv.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] costStateDerivative: Cost State Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, Qv_, modelDataPtr_, ModelData::costStateDerivative);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Cm_: Cm.
     * @param [in] modelDataPtr_: Model Data Ptr.
     * @param [in] stateInputEqConstrStateDerivative: State Input Eq Constr State Derivative.
     * @return Result value.
     */
    ModelData::Interpolate(indexAlpha, Cm_, modelDataPtr_, ModelData::stateInputEqConstrStateDerivative);

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
     * @param [in] costate_: Costate.
     * @param [in] costatePtr_: Costate Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, costate_, costatePtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] lagrangian_: Lagrangian.
     * @param [in] lagrangianPtr_: Lagrangian Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, lagrangian_, lagrangianPtr_);

    /**
     * @brief Time Segment.
     * @param [in] t: T.
     * @param [in] controllerTimeStampPtr_: Controller Time Stamp Ptr.
     * @return Result value.
     */
    indexAlpha = LinearInterpolation::TimeSegment(t, controllerTimeStampPtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] KmConstrained_: Km Constrained.
     * @param [in] KmConstrainedPtr_: Km Constrained Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, KmConstrained_, KmConstrainedPtr_);
    /**
     * @brief Interpolate.
     * @param [in] indexAlpha: Index Alpha.
     * @param [in] Sm_: Sm.
     * @param [in] SmPtr_: Sm Ptr.
     * @return Result value.
     */
    LinearInterpolation::Interpolate(indexAlpha, Sm_, SmPtr_);

    // here we have used RmConstrained = (I-DmConstrained).transpose() * Rm
    // and Km = -(I-DmConstrained) \tilde{L} - CmProjected_
    /**
     * @brief Transpose.
     * @param [in,out] Fv_: Fv.
     * @return Result value.
     */
    dMvdz = (AmConstrained_ + Bm_ * (CmProjected_ + KmConstrained_)).transpose() * Mv +
            /**
             * @brief Transpose.
             * @param [in,out] Fv_: Fv.
             * @return Result value.
             */
            multiplier_ * (Qv_ + Am_.transpose() * costate_ + Cm_.transpose() * lagrangian_ + Sm_ * Fv_);
  }

 private:
  scalar_t multiplier_ = 0.0;

  const scalar_array_t* timeStampPtr_;
  const ModelData::array_t* modelDataPtr_;
  const ModelData::array_t* projectedModelDataPtr_;
  const state_vector_array_t* costatePtr_;
  const dynamic_vector_array_t* lagrangianPtr_;
  const scalar_array_t* controllerTimeStampPtr_;
  const input_state_matrix_array_t* KmConstrainedPtr_;
  const state_matrix_array_t* SmPtr_;

  dynamic_vector_t Fv_;
  dynamic_matrix_t Am_;
  dynamic_matrix_t Bm_;
  dynamic_vector_t Qv_;
  dynamic_matrix_t Cm_;
  dynamic_matrix_t AmConstrained_;
  dynamic_matrix_t CmProjected_;
  state_vector_t costate_;
  dynamic_vector_t lagrangian_;
  input_state_matrix_t KmConstrained_;
  state_matrix_t Sm_;
};

}  // namespace bolza
