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
 * @file target_trajectories.hpp
 * @brief This class is an interface class for the user defined target trajectories.
 */


#include <ostream>

#include "bolza/core/types.hpp"

namespace bolza {

/**
 * This class is an interface class for the user defined target trajectories.
 */
struct TargetTrajectories {
  /**
   * @brief Constructs a TargetTrajectories.
   * @param [in] size: Size.
   */
  explicit TargetTrajectories(size_t size = 0);
  /**
   * @brief Constructs a TargetTrajectories.
   * @param [in] desiredTimeTrajectory: Desired Time Trajectory.
   * @param [in] desiredStateTrajectory: Desired State Trajectory.
   * @param [in] desiredInputTrajectory: Desired Input Trajectory.
   */
  TargetTrajectories(scalar_array_t desiredTimeTrajectory, vector_array_t desiredStateTrajectory,
                     vector_array_t desiredInputTrajectory = vector_array_t());
  /**
   * @brief Clear.
   */
  void clear();
  /**
   * @brief Empty.
   * @return True on success.
   */
  bool empty() const { /**
  * @brief Empty.
  * @return Result value.
  */
 return timeTrajectory.empty() || stateTrajectory.empty(); }
  /**
   * @brief Size.
   * @return Result value.
   */
  size_t size() const { /**
  * @brief Size.
  * @return Result value.
  */
 return timeTrajectory.size(); }

  bool operator==(const TargetTrajectories& other);
  bool operator!=(const TargetTrajectories& other) { return !(*this == other); }

  /**
   * @brief Get Desired State.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector_t GetDesiredState(scalar_t time) const;
  /**
   * @brief Get Desired Input.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector_t GetDesiredInput(scalar_t time) const;

  scalar_array_t timeTrajectory;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
};

/**
 * @brief Swap.
 * @param [in,out] lh: Lh.
 * @param [in,out] rh: Rh.
 */
void swap(TargetTrajectories& lh, TargetTrajectories& rh);
std::ostream& operator<<(std::ostream& out, const TargetTrajectories& targetTrajectories);

}  // namespace bolza
