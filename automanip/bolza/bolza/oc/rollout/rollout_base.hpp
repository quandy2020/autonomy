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
 * @file rollout_base.hpp
 * @brief This class is an interface class for forward rollout of the system dynamics.
 */


#include <utility>
#include <vector>

#include <bolza/core/types.hpp>
#include <bolza/core/control/controller_base.hpp>
#include "bolza/core/reference/mode_schedule.hpp"

#include "bolza/oc/rollout/rollout_settings.hpp"

namespace bolza {

/**
 * This class is an interface class for forward rollout of the system dynamics.
 */
class RolloutBase {
 public:
  /**
   * Default constructor.
   *
   * @param [in] rolloutSettings: The rollout settings.
   */
  explicit RolloutBase(rollout::Settings rolloutSettings) : rolloutSettings_(std::move(rolloutSettings)) {}

  /**
   * Default destructor.
   */
  virtual ~RolloutBase() = default;

  /**
   * Returns the rollout settings.
   *
   * @return The rollout settings.
   */
  const rollout::Settings& Settings() const { return rolloutSettings_; }

  /**
   * The kills the integrator inside the rollout.
   */
  virtual void AbortRollout() {}

  /**
   * The enables the integrator inside the rollout to start again.
   */
  virtual void ReactivateRollout() {}

  /**
   * Resets the simulator state to the initial state in the next runImpl.
   * @note This is relevant if a physics engine (e.g. RaiSim) is used.
   */
  virtual void ResetRollout() {}

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual RolloutBase* Clone() const = 0;

  /**
   * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
   * to integrate the system dynamics in time period [initTime, finalTime].
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] controller: control policy.
   * @param [in,out] modeSchedule: Defines the sequence of modes and the associated event times. For TimeTriggeredRollout
   *                                this is an input argument while for StateTriggeredRollout this is an output argument.
   * @param [out] timeTrajectory: The time trajectory stamp.
   * @param [out] postEventIndices: Indices containing past-the-end index of events trigger.
   * @param [out] stateTrajectory: The state trajectory.
   * @param [out] inputTrajectory: The control input trajectory.
   *
   * @return The final state (state jump is considered if it took place)
   */
  virtual vector_t Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, ControllerBase* controller,
                       ModeSchedule& modeSchedule, scalar_array_t& timeTrajectory, size_array_t& postEventIndices,
                       vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) = 0;

  /**
   * Prints out the rollout.
   *
   * @param [in] timeTrajectory: The time trajectory stamp.
   * @param [in] postEventIndices: An array of the post-event indices.
   * @param [in] stateTrajectory: The state trajectory.
   * @param [in] inputTrajectory: The control input trajectory.
   */
  static void Display(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices, const vector_array_t& stateTrajectory,
                      const vector_array_t* const inputTrajectory);

 protected:
  /**
   * @brief Extracts an array of the rollout's start and final times for each active mode.
   * @param [in] initTime: Init Time.
   * @param [in] finalTime: Final Time.
   * @param [in] eventTimes: Event Times.
   * @return Result value.
   */
  std::vector<std::pair<scalar_t, scalar_t>> FindActiveModesTimeInterval(scalar_t initTime, scalar_t finalTime,
                                                                         const scalar_array_t& eventTimes) const;

  /**
   * @brief Checks for the numerical stability if rollout::Settings::CheckNumericalStability is true.
   * @param [in] controller: Controller.
   * @param [in] timeTrajectory: Time Trajectory.
   * @param [in] postEventIndices: Post Event Indices.
   * @param [in] stateTrajectory: State Trajectory.
   * @param [in] inputTrajectory: Input Trajectory.
   */
  void CheckNumericalStability(const ControllerBase& controller, const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices,
                               const vector_array_t& stateTrajectory, const vector_array_t& inputTrajectory) const;

  const rollout::Settings rolloutSettings_;
};

}  // namespace bolza
