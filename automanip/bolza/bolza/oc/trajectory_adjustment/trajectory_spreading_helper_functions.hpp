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
 * @file trajectory_spreading_helper_functions.hpp
 * @brief Adjusts in-place a linear controller based on the last changes in mode schedule using a TrajectorySpreading strategy.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/control/linear_controller.hpp>
#include <bolza/core/reference/mode_schedule.hpp>

#include "bolza/oc/oc_data/dual_solution.hpp"
#include "bolza/oc/oc_data/primal_solution.hpp"
#include "bolza/oc/trajectory_adjustment/trajectory_spreading.hpp"

namespace bolza {

/**
 * Adjusts in-place a linear controller based on the last changes in mode schedule using a TrajectorySpreading strategy.
 *
 * @param [in] oldModeSchedule: The old mode schedule associated to the trajectories which should be adjusted.
 * @param [in] newModeSchedule: The new mode schedule that should be adapted to.
 * @param [in,out] controller: The control policy that is associated with the old mode schedule.
 * @returns the status of the devised trajectory spreading strategy.
 */
inline TrajectorySpreading::Status TrajectorySpread(const ModeSchedule& oldModeSchedule, const ModeSchedule& newModeSchedule,
                                                    LinearController& controller) {
  // trajectory spreading
  constexpr bool debugPrint = false;
  /**
   * @brief Trajectory Spreading.
   * @param [in] debugPrint: Debug Print.
   * @return Result value.
   */
  TrajectorySpreading TrajectorySpreading(debugPrint);
  const auto status = trajectorySpreading.set(oldModeSchedule, newModeSchedule, controller.timeStamp_);

  // adjust bias, gain, and time
  /**
   * @brief Adjust Trajectory.
   * @param [in] biasArray_: Bias Array.
   * @return Result value.
   */
  trajectorySpreading.adjustTrajectory(controller.biasArray_);
  /**
   * @brief Adjust Trajectory.
   * @param [in] gainArray_: Gain Array.
   * @return Result value.
   */
  trajectorySpreading.adjustTrajectory(controller.gainArray_);
  /**
   * @brief Adjust Time Trajectory.
   * @param [in] timeStamp_: Time Stamp.
   * @return Result value.
   */
  trajectorySpreading.adjustTimeTrajectory(controller.timeStamp_);

  return status;
}

/**
 * Adjusts in-place a dual solution based on the last changes in mode schedule using a TrajectorySpreading strategy.
 *
 * @param [in] oldModeSchedule: The old mode schedule associated to the trajectories which should be adjusted.
 * @param [in] newModeSchedule: The new mode schedule that should be adapted to.
 * @param [in,out] dualSolution: The dual solution that is associated with the old mode schedule.
 * @returns the status of the devised trajectory spreading strategy.
 */
inline TrajectorySpreading::Status TrajectorySpread(const ModeSchedule& oldModeSchedule, const ModeSchedule& newModeSchedule,
                                                    DualSolution& dualSolution) {
  // trajectory spreading
  constexpr bool debugPrint = false;
  /**
   * @brief Trajectory Spreading.
   * @param [in] debugPrint: Debug Print.
   * @return Result value.
   */
  TrajectorySpreading TrajectorySpreading(debugPrint);
  const auto status = trajectorySpreading.set(oldModeSchedule, newModeSchedule, dualSolution.timeTrajectory);

  // adjust final, pre-jump, intermediate, time, postEventIndices
  if (status.willTruncate) {
    /**
     * @brief Clear.
     * @return Result value.
     */
    dualSolution.final.clear();
  }
  dualSolution.preJumps = trajectorySpreading.extractEventsArray(dualSolution.preJumps);
  /**
   * @brief Adjust Trajectory.
   * @param [in] intermediates: Intermediates.
   * @return Result value.
   */
  trajectorySpreading.adjustTrajectory(dualSolution.intermediates);
  /**
   * @brief Adjust Time Trajectory.
   * @param [in] timeTrajectory: Time Trajectory.
   * @return Result value.
   */
  trajectorySpreading.adjustTimeTrajectory(dualSolution.timeTrajectory);
  dualSolution.postEventIndices = trajectorySpreading.getPostEventIndices();

  return status;
}

/**
 * Adjusts a dual solution based on the last changes in mode schedule using a TrajectorySpreading strategy.
 *
 * @param [in] trajectorySpreading: An updated trajectorySpreading instance. In order to update trajectorySpreading
 *                                  call TrajectorySpreading::set.
 * @param [in] oldDualSolution: The dual solution that is associated with the old mode schedule.
 * @param [out] newDualSolution: The updated dual solution that is associated with the new mode schedule.
 */
inline void TrajectorySpread(const TrajectorySpreading& trajectorySpreading, const DualSolution& oldDualSolution,
                             DualSolution& newDualSolution) {
  /**
   * @brief Clear.
   * @return Result value.
   */
  newDualSolution.clear();

  // adjust time and postEventIndices
  newDualSolution.timeTrajectory = oldDualSolution.timeTrajectory;
  /**
   * @brief Adjust Time Trajectory.
   * @param [in] timeTrajectory: Time Trajectory.
   * @return Result value.
   */
  trajectorySpreading.adjustTimeTrajectory(newDualSolution.timeTrajectory);
  newDualSolution.postEventIndices = trajectorySpreading.getPostEventIndices();

  // adjust final, pre-jump and intermediate
  if (!trajectorySpreading.getStatus().willTruncate) {
    newDualSolution.final = oldDualSolution.final;
  }
  newDualSolution.preJumps = trajectorySpreading.extractEventsArray(oldDualSolution.preJumps);
  newDualSolution.intermediates = oldDualSolution.intermediates;
  /**
   * @brief Adjust Trajectory.
   * @param [in] intermediates: Intermediates.
   * @return Result value.
   */
  trajectorySpreading.adjustTrajectory(newDualSolution.intermediates);
}

/**
 * Adjusts in-place a primal solution based on the last changes in mode schedule using a TrajectorySpreading strategy.
 * Note: PrimalSolution::controllerPtr_ will not be adjusted.
 *
 * @param [in] oldModeSchedule: The old mode schedule associated to the trajectories which should be adjusted.
 * @param [in] newModeSchedule: The new mode schedule that should be adapted to.
 * @param [in,out] primalSolution: The primal solution that is associated with the old mode schedule.
 * @returns the status of the devised trajectory spreading strategy.
 */
inline TrajectorySpreading::Status TrajectorySpread(const ModeSchedule& oldModeSchedule, const ModeSchedule& newModeSchedule,
                                                    PrimalSolution& primalSolution) {
  // trajectory spreading
  constexpr bool debugPrint = false;
  /**
   * @brief Trajectory Spreading.
   * @param [in] debugPrint: Debug Print.
   * @return Result value.
   */
  TrajectorySpreading TrajectorySpreading(debugPrint);
  const auto status = trajectorySpreading.set(oldModeSchedule, newModeSchedule, primalSolution.timeTrajectory_);

  // adjust modeSchedule, state, input, time, postEventIndices
  primalSolution.modeSchedule_ = newModeSchedule;
  /**
   * @brief Adjust Trajectory.
   * @param [in] stateTrajectory_: State Trajectory.
   * @return Result value.
   */
  trajectorySpreading.adjustTrajectory(primalSolution.stateTrajectory_);
  /**
   * @brief Adjust Trajectory.
   * @param [in] inputTrajectory_: Input Trajectory.
   * @return Result value.
   */
  trajectorySpreading.adjustTrajectory(primalSolution.inputTrajectory_);
  /**
   * @brief Adjust Time Trajectory.
   * @param [in] timeTrajectory_: Time Trajectory.
   * @return Result value.
   */
  trajectorySpreading.adjustTimeTrajectory(primalSolution.timeTrajectory_);
  primalSolution.postEventIndices_ = trajectorySpreading.getPostEventIndices();

  return status;
}

}  // namespace bolza
