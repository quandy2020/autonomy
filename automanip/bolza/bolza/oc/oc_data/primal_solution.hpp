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
 * @file primal_solution.hpp
 * @brief This class contains the primal problem's solution.
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/control/controller_base.hpp>
#include <bolza/core/reference/mode_schedule.hpp>

namespace bolza {

/**
 * This class contains the primal problem's solution.
 */
struct PrimalSolution {
  /** Constructor */
  PrimalSolution() = default;

  /** Destructor */
  ~PrimalSolution() = default;

  /** Copy constructor */
  PrimalSolution(const PrimalSolution& other)
      /**
       * @brief Time Trajectory.
       * @param [in] nullptr: Nullptr.
       * @return Result value.
       */
      : timeTrajectory_(other.timeTrajectory_),
        stateTrajectory_(other.stateTrajectory_),
        inputTrajectory_(other.inputTrajectory_),
        postEventIndices_(other.postEventIndices_),
        modeSchedule_(other.modeSchedule_),
        controllerPtr_(other.controllerPtr_ ? other.controllerPtr_->clone() : nullptr) {}

  /** Copy Assignment */
  PrimalSolution& operator=(const PrimalSolution& other) {
    timeTrajectory_ = other.timeTrajectory_;
    stateTrajectory_ = other.stateTrajectory_;
    inputTrajectory_ = other.inputTrajectory_;
    postEventIndices_ = other.postEventIndices_;
    modeSchedule_ = other.modeSchedule_;
    if (other.controllerPtr_) {
      /**
       * @brief Reset.
       * @return Result value.
       */
      controllerPtr_.reset(other.controllerPtr_->clone());
    } else {
      /**
       * @brief Reset.
       * @return Result value.
       */
      controllerPtr_.reset();
    }
    return *this;
  }

  /**
   * @brief Move constructor.
   * @param [in,out] other: Other.
   */
  PrimalSolution(PrimalSolution&& other) noexcept = default;

  /** Move Assignment */
  PrimalSolution& operator=(PrimalSolution&& other) noexcept = default;

  /**
   * @brief Swap.
   * @param [in,out] other: Other.
   */
  void swap(PrimalSolution& other) {
    /**
     * @brief Swap.
     * @param [in] timeTrajectory_: Time Trajectory.
     * @return Result value.
     */
    timeTrajectory_.swap(other.timeTrajectory_);
    /**
     * @brief Swap.
     * @param [in] stateTrajectory_: State Trajectory.
     * @return Result value.
     */
    stateTrajectory_.swap(other.stateTrajectory_);
    /**
     * @brief Swap.
     * @param [in] inputTrajectory_: Input Trajectory.
     * @return Result value.
     */
    inputTrajectory_.swap(other.inputTrajectory_);
    /**
     * @brief Swap.
     * @param [in] postEventIndices_: Post Event Indices.
     * @return Result value.
     */
    postEventIndices_.swap(other.postEventIndices_);
    /**
     * @brief Swap.
     * @param [in] modeSchedule_: Mode Schedule.
     * @param [in] modeSchedule_: Mode Schedule.
     * @return Result value.
     */
    ::bolza::swap(modeSchedule_, other.modeSchedule_);
    /**
     * @brief Swap.
     * @param [in] controllerPtr_: Controller Ptr.
     * @return Result value.
     */
    controllerPtr_.swap(other.controllerPtr_);
  }

  /**
   * @brief Clear.
   */
  void clear() {
    /**
     * @brief Clear.
     * @return Result value.
     */
    timeTrajectory_.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    stateTrajectory_.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    inputTrajectory_.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    postEventIndices_.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    modeSchedule_.clear();
    if (controllerPtr_ != nullptr) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      controllerPtr_->clear();
    }
  }

  scalar_array_t timeTrajectory_;
  vector_array_t stateTrajectory_;
  vector_array_t inputTrajectory_;
  size_array_t postEventIndices_;
  ModeSchedule modeSchedule_;
  std::unique_ptr<ControllerBase> controllerPtr_;
};

}  // namespace bolza
