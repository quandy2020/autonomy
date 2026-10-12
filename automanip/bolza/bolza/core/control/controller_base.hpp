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
 * @file controller_base.hpp
 * @brief The base class for all controllers.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/control/controller_type.hpp>

namespace bolza {

/**
 * The base class for all controllers.
 */
class ControllerBase {
 public:
  /** Constructor */
  ControllerBase() = default;

  /** Default destructor. */
  virtual ~ControllerBase() = default;

  /**
   * @brief Computes the control command at a given time and state.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return Current input.
   */
  virtual vector_t ComputeInput(scalar_t t, const vector_t& x) = 0;

  /**
   * @brief Merges this controller with another controller that comes active later in time
   * This method is typically used to merge controllers from multiple time partitions.
   * Indices specifying a range of elements. Copies of the elements in the range [index, index_length) are inserted at the end.
   *
   * @note Only controllers of the same type can be merged.
   *
   * @param [in] otherController: The control law to be appended.
   * @param [in] index: The starting index
   * @param [in] length: The length of the copy.
   */
  virtual void Concatenate(const ControllerBase* otherController, int index, int length) = 0;

  /**
   * @brief Merges this controller with another controller that comes active later in time
   * This method is typically used to merge controllers from multiple time partitions.
   * @note Only controllers of the same type can be merged.
   *
   * @param [in] otherController: The control law to be appended.
   */
  void Concatenate(const ControllerBase* otherController) { /**
  * @brief Constructs a Concatenate.
  * @param [in] otherController: Other Controller.
  */
 Concatenate(otherController, 0, otherController->size()); }

  /**
   * @brief Returns the size of the controller.
   *
   * @return The size of the controller.
   */
  virtual int size() const = 0;

  /**
   * @brief Prints the type of controller
   * @return ControllerType: what type of controller this is
   */
  virtual ControllerType GetType() const = 0;

  /**
   * @brief clears and reverts back to an empty controller.
   * Therefore, if empty() method is called, it will return true.
   */
  virtual void clear() = 0;

  /**
   * Returns whether the class contains any information.
   *
   * @return true if it contains no information, false otherwise.
   */
  virtual bool empty() const = 0;

  /**
   * @brief Create a deep copy of the object.
   * @warning Cloning implies that the caller takes ownership and deletes the created object.
   * @return Pointer to a new instance.
   */
  virtual ControllerBase* Clone() const = 0;

  /**
   * Displays controller's data.
   */
  virtual void Display() const {}

  /**
   * @brief Gets the event times for which the controller is designed.
   * @return The event times of the controller.
   */
  virtual scalar_array_t ControllerEventTimes() const { return {}; }

  /**
   * Saves the controller at given time to an array of arrays structure for ROS transmission
   *
   * @param [in] timeArray: array of query times
   * @param [out] flatArray2: The array of arrays that is to be filled, i.e., the compressed controller. One array per query time
   */
  virtual void Flatten(const scalar_array_t& timeArray, const std::vector<std::vector<float>*>& flatArray2) const {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("ControllerBase::Flatten: not implemented.");
  }

 protected:
    /**
   * @brief /** Copy constructor */.
   * @param [in] rhs: Rhs.
   */
  ControllerBase(const ControllerBase& rhs) = default;
};

}  // namespace bolza
