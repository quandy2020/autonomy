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
 * @file state_based_linear_controller.hpp
 * @brief Default constructor.
 */


#include <bolza/core/control/controller_base.hpp>
#include <bolza/core/control/linear_controller.hpp>

namespace bolza {

class StateBasedLinearController final : public ControllerBase {
 public:
  /** Default constructor */
  StateBasedLinearController() = default;

  /** Default destructor */
  ~StateBasedLinearController() override = default;

  /**
   * Sets the provided controller pointer which provides the input signal
   * At the same time the eventtimes of the designed controller are computed
   *
   * @param [in] ctrlPtr: pointer to the provided controller
   */
  void SetController(ControllerBase* ctrlPtr);

    /**
   * @brief /**.
   *
   * Computes the control input based on the trajectory spreading scheme.
   *
   * /
   * @param [in] t: current time at which input is requested
   * @param [in] x: current state at which input is requested
   * @param [in] ctrlEventTimes: array containing eventTimes around which the controller was designed
   * @param [in] ctrlPtr: Pointer to the actual controller
   * @return Result value.
   */
  static vector_t ComputeTrajectorySpreadingInput(scalar_t t, const vector_t& x, const scalar_array_t& ctrlEventTimes,
                                                  ControllerBase* ctrlPtr);

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
   * @brief Clone.
   * @return Pointer; may be null.
   */
  StateBasedLinearController* Clone() const override;

 private:
  ControllerBase* ctrlPtr_ = nullptr;
  scalar_array_t ctrlEventTimes_{0};
};

}  // namespace bolza
