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
 * @file linear_controller.hpp
 * @brief LinearController implements a time and state dependent controller of the.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/control/controller_base.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>

namespace bolza {

/**
 * LinearController implements a time and state dependent controller of the
 * form u[x,t] = k[t] * x + uff[t]
 */
class LinearController final : public ControllerBase {
 public:
  /** Constructor, leaves object uninitialized */
  LinearController() = default;

  /**
   * @brief Constructor initializes all required members of the controller.
   *
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerBias: The bias array.
   * @param [in] controllerGain: The feedback gain array.
   */
  LinearController(scalar_array_t controllerTime, vector_array_t controllerBias, matrix_array_t controllerGain)
      /**
       * @brief Time Stamp.
       * @return Result value.
       */
      : timeStamp_(std::move(controllerTime)), biasArray_(std::move(controllerBias)), gainArray_(std::move(controllerGain)) {}

    /**
   * @brief Copy constructor 
   * @param [in] other: Other.
   */
  LinearController(const LinearController& other);

    /**
   * @brief Move constructor 
   * @param [in,out] other: Other.
   */
  LinearController(LinearController&& other);

  /** Copy assignment (copy and swap idiom) */
  LinearController& operator=(LinearController rhs);

  /** Destructor */
  ~LinearController() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LinearController* Clone() const override;

  /**
   * @brief setController Assign control law
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerBias: The bias array.
   * @param [in] controllerGain: The feedback gain array.
   */
  void SetController(const scalar_array_t& controllerTime, const vector_array_t& controllerBias, const matrix_array_t& controllerGain);

  /**
   * @brief Compute Input.
   * @param [in] t: T.
   * @param [in] x: X.
   * @return Result value.
   */
  vector_t ComputeInput(scalar_t t, const vector_t& x) override;

  /**
   * @brief Concatenate.
   * @param [in] nextController: Next Controller.
   * @param [in] index: Index.
   * @param [in] length: Length.
   */
  void Concatenate(const ControllerBase* nextController, int index, int length) override;

  /**
   * @brief Size.
   * @return Result value.
   */
  int size() const override;

  /**
   * @brief Get Type.
   * @return Result value.
   */
  ControllerType GetType() const override;

  /**
   * @brief Clear.
   */
  void clear() override;

  /**
   * @brief Empty.
   * @return True on success.
   */
  bool empty() const override;

  /**
   * @brief Display.
   */
  void Display() const override;

  /**
   * @brief Get Feedback Gain.
   * @param [in] time: Time.
   * @param [in,out] gain: Gain.
   */
  void GetFeedbackGain(scalar_t time, matrix_t& gain) const;

  /**
   * @brief Get Bias.
   * @param [in] time: Time.
   * @param [in,out] bias: Bias.
   */
  void GetBias(scalar_t time, vector_t& bias) const;

  /**
   * @brief Controller Event Times.
   * @return Result value.
   */
  scalar_array_t ControllerEventTimes() const override;

  /**
   * @brief Flatten.
   * @param [in] timeArray: Time Array.
   * @param [in] flatArray2: Flat Array2.
   */
  void Flatten(const scalar_array_t& timeArray, const std::vector<std::vector<float>*>& flatArray2) const override;

  /**
   * @brief Un Flatten.
   * @param [in] stateDim: State Dim.
   * @param [in] inputDim: Input Dim.
   * @param [in] timeArray: Time Array.
   * @param [in] flatArray2: Flat Array2.
   * @return Result value.
   */
  static LinearController UnFlatten(const size_array_t& stateDim, const size_array_t& inputDim, const scalar_array_t& timeArray,
                                    const std::vector<std::vector<float> const*>& flatArray2);

 private:
  /**
   * @brief Flatten Single.
   * @param [in] time: Time.
   * @param [in,out] flatArray: Flat Array.
   */
  void FlattenSingle(scalar_t time, std::vector<float>& flatArray) const;

 public:
  scalar_array_t timeStamp_;
  vector_array_t biasArray_;
  vector_array_t deltaBiasArray_;
  matrix_array_t gainArray_;

  /**
   * @brief Swap.
   * @param [in,out] a: A.
   * @param [in,out] b: B.
   */
  friend void swap(LinearController& a, LinearController& b) noexcept;
};

/**
 * @brief Swap.
 * @param [in,out] a: A.
 * @param [in,out] b: B.
 */
void swap(LinearController& a, LinearController& b) noexcept;

std::ostream& operator<<(std::ostream& out, const LinearController& controller);

}  // namespace bolza
