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
 * @file mpcnet_rollout_base.hpp
 * @brief The base class for doing rollouts for a system that is forward simulated with a behavioral controller.
 */


#include <bolza/core/reference/mode_schedule.hpp>
#include <bolza/core/reference/target_trajectories.hpp>
#include <bolza/mpc/mpc_base.hpp>
#include <bolza/mpc/system_observation.hpp>
#include <bolza/oc/oc_data/primal_solution.hpp>
#include <bolza/oc/rollout/rollout_base.hpp>
#include <bolza/oc/synchronized_module/reference_manager_interface.hpp>

#include "bolza/mpcnet/mpcnet_definition_base.hpp"
#include "bolza/mpcnet/control/mpcnet_behavioral_controller.hpp"
#include "bolza/mpcnet/control/mpcnet_controller_base.hpp"

namespace bolza {
namespace mpcnet {

/**
 *  The base class for doing rollouts for a system that is forward simulated with a behavioral controller.
 *  The behavioral policy is a mixture of an MPC policy and an MPC-Net policy (e.g. a neural network).
 */
class MPCNetRolloutBase {
 public:
  /**
   * Constructor.
   * @param [in] mpcPtr: Pointer to the MPC solver to be used (this class takes ownership).
   * @param [in] mpcnetPtr: Pointer to the MPC-Net policy to be used (this class takes ownership).
   * @param [in] rolloutPtr: Pointer to the rollout to be used (this class takes ownership).
   * @param [in] mpcnetDefinitionPtr: Pointer to the MPC-Net definitions to be used (shared ownership).
   * @param [in] referenceManagerPtr: Pointer to the reference manager to be used (shared ownership).
   */
  MPCNetRolloutBase(std::unique_ptr<MPCBase> mpcPtr, std::unique_ptr<MPCNetControllerBase> mpcnetPtr,
                    std::unique_ptr<RolloutBase> rolloutPtr, std::shared_ptr<MPCNetDefinitionBase> mpcnetDefinitionPtr,
                    std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
      /**
       * @brief Mpc Ptr.
       * @return Result value.
       */
      : mpcPtr_(std::move(mpcPtr)),
        mpcnetPtr_(std::move(mpcnetPtr)),
        rolloutPtr_(std::move(rolloutPtr)),
        mpcnetDefinitionPtr_(std::move(mpcnetDefinitionPtr)),
        referenceManagerPtr_(std::move(referenceManagerPtr)),
        behavioralControllerPtr_(new MPCNetBehavioralController()) {}

  /**
   * Default destructor.
   */
  virtual ~MPCNetRolloutBase() = default;

  /**
   * Deleted copy constructor.
   */
  MPCNetRolloutBase(const MPCNetRolloutBase&) = delete;

  /**
   * Deleted copy assignment.
   */
  MPCNetRolloutBase& operator=(const MPCNetRolloutBase&) = delete;

 protected:
  /**
   * (Re)set system components.
   * @param [in] alpha: The mixture parameter for the behavioral controller.
   * @param [in] policyFilePath: The path to the file with the learned policy for the controller.
   * @param [in] initialObservation: The initial system observation to start from (time and state required).
   * @param [in] modeSchedule: The mode schedule providing the event times and mode sequence.
   * @param [in] targetTrajectories: The target trajectories to be tracked.
   */
  void set(scalar_t alpha, const std::string& policyFilePath, const SystemObservation& initialObservation, const ModeSchedule& modeSchedule,
           const TargetTrajectories& targetTrajectories);

  /**
   * Simulate the system one step forward.
   * @param [in] timeStep: The time step for the forward simulation of the system with the behavioral controller.
   */
  void Step(scalar_t timeStep);

  std::unique_ptr<MPCBase> mpcPtr_;
  std::shared_ptr<MPCNetDefinitionBase> mpcnetDefinitionPtr_;
  std::unique_ptr<MPCNetBehavioralController> behavioralControllerPtr_;
  SystemObservation systemObservation_;
  PrimalSolution primalSolution_;

 private:
  std::unique_ptr<MPCNetControllerBase> mpcnetPtr_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;
};

}  // namespace mpcnet
}  // namespace bolza
