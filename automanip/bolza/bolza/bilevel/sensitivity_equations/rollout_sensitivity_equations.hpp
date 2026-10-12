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
 * @file rollout_sensitivity_equations.hpp
 * @brief Rollout sensitivity equations class.
 */


#include <Eigen/Dense>

#include <array>

#include <bolza/core/dimensions.hpp>
#include <bolza/core/control/linear_controller.hpp>
#include <bolza/core/dynamics/controlled_system_base.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/core/misc/numerics.hpp>
#include <bolza/core/model_data/model_data_linear_interpolation.hpp>

namespace bolza {

/**
 * Rollout sensitivity equations class
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class RolloutSensitivityEquations final : public ControlledSystemBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ControlledSystemBase<STATE_DIM, INPUT_DIM>;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using state_input_matrix_array_t = typename DIMENSIONS::state_input_matrix_array_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;

  using linear_controller_t = LinearController<STATE_DIM, INPUT_DIM>;

  /**
   * The default constructor.
   */
  RolloutSensitivityEquations() = default;

  /**
   * Default destructor.
   */
  ~RolloutSensitivityEquations() override = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  RolloutSensitivityEquations<STATE_DIM, INPUT_DIM>* Clone() const override {
    return new RolloutSensitivityEquations<STATE_DIM, INPUT_DIM>(*this);
  }

    /**
   * @brief /**.
   *
   * Sets Data
   *
   * /
   * @param [in] timeTrajectoryPtr: Time Trajectory Ptr.
   * @param [in] modelDataPtr: Model Data Ptr.
   * @param [in] sensitivityControllerTimePtr: Sensitivity Controller Time Ptr.
   * @param [in] sensitivityControllerFeedforwardPtr: Sensitivity Controller Feedforward Ptr.
   * @param [in] sensitivityControllerFeedbackPtr: Sensitivity Controller Feedback Ptr.
   */
  void SetData(const scalar_array_t* timeTrajectoryPtr, const ModelData::array_t* modelDataPtr,
               const scalar_array_t* sensitivityControllerTimePtr, const input_vector_array_t* sensitivityControllerFeedforwardPtr,
               const input_state_matrix_array_t* sensitivityControllerFeedbackPtr) {
    timeTrajectoryPtr_ = timeTrajectoryPtr;
    modelDataPtr_ = modelDataPtr;
    /**
     * @brief Set Controller.
     * @param [in,out] sensitivityControllerTimePtr: Sensitivity Controller Time Ptr.
     * @param [in,out] sensitivityControllerFeedforwardPtr: Sensitivity Controller Feedforward Ptr.
     * @param [in,out] sensitivityControllerFeedbackPtr: Sensitivity Controller Feedback Ptr.
     * @return Result value.
     */
    linearController_.setController(*sensitivityControllerTimePtr, *sensitivityControllerFeedforwardPtr, *sensitivityControllerFeedbackPtr);
    /**
     * @brief Set Controller.
     * @param [in,out] linearController_: Linear Controller.
     * @return Result value.
     */
    this->setController(&linearController_);
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
   * @param [in] t: time
   * @param [in] nabla_x: Nabla x.
   * @param [in] nabla_u: Nabla u.
   * @param [in,out] derivative: Derivative.
   */
  void ComputeFlowMap(const scalar_t& t, const state_vector_t& nabla_x, const input_vector_t& nabla_u,
                      state_vector_t& derivative) override {
    /**
     * @brief Time Segment.
     * @param [in] t: T.
     * @param [in] timeTrajectoryPtr_: Time Trajectory Ptr.
     * @return Result value.
     */
    const auto indexAlpha = LinearInterpolation::TimeSegment(t, timeTrajectoryPtr_);

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

    if (!numerics::almost_eq(multiplier_, 0.0)) {
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] Fv_: Fv.
       * @param [in] modelDataPtr_: Model Data Ptr.
       * @param [in] dynamics: Dynamics.
       * @return Result value.
       */
      ModelData::Interpolate(indexAlpha, Fv_, modelDataPtr_, ModelData::dynamics);
      derivative = Am_ * nabla_x + Bm_ * nabla_u + multiplier_ * Fv_;
    } else {
      derivative = Am_ * nabla_x + Bm_ * nabla_u;
    }
  }

 protected:
  scalar_t multiplier_ = 0.0;
  const scalar_array_t* timeTrajectoryPtr_;
  const ModelData::array_t* modelDataPtr_;
  linear_controller_t linearController_;

  dynamic_vector_t Fv_;
  dynamic_matrix_t Am_;
  dynamic_matrix_t Bm_;
};

}  // namespace bolza
