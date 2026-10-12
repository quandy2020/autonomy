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
 * @file initializer_rollout.hpp
 * @brief This class is an interface class for forward rollout of the initializer.
 */


#include <memory>

#include <bolza/core/initialization/initializer.hpp>

#include "bolza/oc/rollout/rollout_base.hpp"

namespace bolza {

/**
 * This class is an interface class for forward rollout of the initializer.
 */
class InitializerRollout : public RolloutBase {
 public:
  /**
   * Constructor.
   *
   * @param [in] initializer: The initializer for the state and the input.
   * @param [in] rolloutSettings: The rollout settings.
   */
  explicit InitializerRollout(const Initializer& initializer, rollout::Settings rolloutSettings = rollout::Settings());

  /**
   * @brief Destructor.
   */
  ~InitializerRollout() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  InitializerRollout* Clone() const override;

  /**
   * @brief Run.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   * @param [in,out] controller: Controller.
   * @param [in,out] modeSchedule: Mode Schedule.
   * @param [in,out] timeTrajectory: Time Trajectory.
   * @param [in,out] postEventIndices: Post Event Indices.
   * @param [in,out] stateTrajectory: State Trajectory.
   * @param [in,out] inputTrajectory: Input Trajectory.
   * @return Result value.
   */
  vector_t Run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, ControllerBase* controller, ModeSchedule& modeSchedule,
               scalar_array_t& timeTrajectory, size_array_t& postEventIndices, vector_array_t& stateTrajectory,
               vector_array_t& inputTrajectory) override;

 private:
  std::unique_ptr<Initializer> initializerPtr_;
};

}  // namespace bolza
