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
 * @file mpcnet_definition_base.hpp
 * @brief Base class for MPC-Net definitions.
 */


#include <bolza/core/reference/mode_schedule.hpp>
#include <bolza/core/reference/target_trajectories.hpp>

namespace bolza {
namespace mpcnet {

/**
 * Base class for MPC-Net definitions.
 */
class MPCNetDefinitionBase {
 public:
  /**
   * Default constructor.
   */
  MPCNetDefinitionBase() = default;

  /**
   * Default destructor.
   */
  virtual ~MPCNetDefinitionBase() = default;

  /**
   * Deleted copy constructor.
   */
  MPCNetDefinitionBase(const MPCNetDefinitionBase&) = delete;

  /**
   * Deleted copy assignment.
   */
  MPCNetDefinitionBase& operator=(const MPCNetDefinitionBase&) = delete;

  /**
   * Get the observation.
   * @note The observation o is the input to the policy.
   * @param [in] t: Absolute time.
   * @param [in] x: Robot state.
   * @param [in] modeSchedule: Mode schedule.
   * @param [in] targetTrajectories: Target trajectories.
   * @return The observation.
   */
  virtual vector_t GetObservation(scalar_t t, const vector_t& x, const ModeSchedule& modeSchedule,
                                  const TargetTrajectories& targetTrajectories) = 0;

  /**
   * Get the action transformation.
   * @note Used for computing the control input u = A * a + b from the action a predicted by the policy.
   * @param [in] t: Absolute time.
   * @param [in] x: Robot state.
   * @param [in] modeSchedule: Mode schedule.
   * @param [in] targetTrajectories: Target trajectories.
   * @return The action transformation pair {A, b}.
   */
  virtual std::pair<matrix_t, vector_t> GetActionTransformation(scalar_t t, const vector_t& x, const ModeSchedule& modeSchedule,
                                                                const TargetTrajectories& targetTrajectories) = 0;

  /**
   * Check if the tuple (t, x, modeSchedule, targetTrajectories) is valid.
   * @note E.g., check if the state diverged or if tracking is poor.
   * @param [in] t: Absolute time.
   * @param [in] x: Robot state.
   * @param [in] modeSchedule: Mode schedule.
   * @param [in] targetTrajectories: Target trajectories.
   * @return True if valid.
   */
  virtual bool IsValid(scalar_t t, const vector_t& x, const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories) = 0;
};

}  // namespace mpcnet
}  // namespace bolza
