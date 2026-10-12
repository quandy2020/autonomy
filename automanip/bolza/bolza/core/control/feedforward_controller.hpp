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
 * @file feedforward_controller.hpp
 * @brief FeedforwardController provides a time-dependent control law without state-dependent feedback.
 */


#include <ostream>

#include <bolza/core/types.hpp>
#include <bolza/core/control/controller_base.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>

namespace bolza {

/**
 * FeedforwardController provides a time-dependent control law without state-dependent feedback.
 * Commonly, this is used to wrap around a more general controller and extract only the feedforward portion.
 */
class FeedforwardController final : public ControllerBase {
 public:
  /** Constructor, leaves object uninitialized */
  FeedforwardController() = default;

  /**
   * Constructor initializes all required members of the controller.
   *
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerFeedforward: The feedforward control input array.
   */
  FeedforwardController(scalar_array_t controllerTime, vector_array_t controllerFeedforward)
      /**
       * @brief Time Stamp.
       * @return Result value.
       */
      : timeStamp_(std::move(controllerTime)), uffArray_(std::move(controllerFeedforward)) {}

  /**
   * Constructor to initialize the feedforward input data with a general controller rolled-out along a nominal stateTrajectory
   *
   * @param [in] controllerTime: the times for the rollout
   * @param [in] stateTrajectory: the states for the rollout
   * @param [in] controller: the controller to extract the feedforward controls from during a rollout
   */
  FeedforwardController(const scalar_array_t& controllerTime, const vector_array_t& stateTrajectory, ControllerBase* controller);

    /**
   * @brief /** Copy constructor */.
   * @param [in] other: Other.
   */
  FeedforwardController(const FeedforwardController& other);

    /**
   * @brief /** Move constructor */.
   * @param [in,out] other: Other.
   */
  FeedforwardController(FeedforwardController&& other);

  /** Copy assignment (copy and swap idiom) */
  FeedforwardController& operator=(FeedforwardController rhs);

  /** Destructor */
  ~FeedforwardController() override = default;

  /**
   * setController Assign control law
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerFeedforward: The feedforward control input array.
   */
  void SetController(const scalar_array_t& controllerTime, const vector_array_t& controllerFeedforward);

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
   * @brief Clone.
   * @return Pointer; may be null.
   */
  FeedforwardController* Clone() const override;

  /**
   * @brief Display.
   */
  void Display() const override;

  /**
   * @brief Flatten.
   * @param [in] timeArray: Time Array.
   * @param [in] flatArray2: Flat Array2.
   */
  void Flatten(const scalar_array_t& timeArray, const std::vector<std::vector<float>*>& flatArray2) const override;

  /**
   * @brief Un Flatten.
   * @param [in] timeArray: Time Array.
   * @param [in] flatArray2: Flat Array2.
   * @return Result value.
   */
  static FeedforwardController UnFlatten(const scalar_array_t& timeArray, const std::vector<std::vector<float> const*>& flatArray2);

 private:
  /**
   * @brief Flatten Single.
   * @param [in] time: Time.
   * @param [in,out] flatArray: Flat Array.
   */
  void FlattenSingle(scalar_t time, std::vector<float>& flatArray) const;

 public:
  scalar_array_t timeStamp_;
  vector_array_t uffArray_;

  /**
   * @brief Swap.
   * @param [in,out] a: A.
   * @param [in,out] b: B.
   */
  friend void swap(FeedforwardController& a, FeedforwardController& b) noexcept;
};

/**
 * @brief Swap.
 * @param [in,out] a: A.
 * @param [in,out] b: B.
 */
void swap(FeedforwardController& a, FeedforwardController& b) noexcept;

std::ostream& operator<<(std::ostream& out, const FeedforwardController& controller);

}  // namespace bolza
