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
 * @file mpcnet_policy_evaluation.hpp
 * @brief A class for evaluating a policy for a system that is forward simulated with a behavioral controller.
 */


#include "bolza/mpcnet/rollout/mpcnet_metrics.hpp"
#include "bolza/mpcnet/rollout/mpcnet_rollout_base.hpp"

namespace bolza {
namespace mpcnet {

/**
 *  A class for evaluating a policy for a system that is forward simulated with a behavioral controller.
 *  @note Usually the behavioral controller is evaluated for the MPC-Net policy (alpha = 0).
 */
class MPCNetPolicyEvaluation final : public MPCNetRolloutBase {
 public:
  /**
   * Constructor.
   * @param [in] mpcPtr: Pointer to the MPC solver to be used (this class takes ownership).
   * @param [in] mpcnetPtr: Pointer to the MPC-Net policy to be used (this class takes ownership).
   * @param [in] rolloutPtr: Pointer to the rollout to be used (this class takes ownership).
   * @param [in] mpcnetDefinitionPtr: Pointer to the MPC-Net definitions to be used (shared ownership).
   * @param [in] referenceManagerPtr: Pointer to the reference manager to be used (shared ownership).
   */
  MPCNetPolicyEvaluation(std::unique_ptr<MPCBase> mpcPtr, std::unique_ptr<MPCNetControllerBase> mpcnetPtr,
                         std::unique_ptr<RolloutBase> rolloutPtr, std::shared_ptr<MPCNetDefinitionBase> mpcnetDefinitionPtr,
                         std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
      /**
       * @brief MPCNet Rollout Base.
       * @return Result value.
       */
      : MPCNetRolloutBase(std::move(mpcPtr), std::move(mpcnetPtr), std::move(rolloutPtr), std::move(mpcnetDefinitionPtr),
                          std::move(referenceManagerPtr)) {}

  /**
   * Default destructor.
   */
  ~MPCNetPolicyEvaluation() override = default;

  /**
   * Deleted copy constructor.
   */
  MPCNetPolicyEvaluation(const MPCNetPolicyEvaluation&) = delete;

  /**
   * Deleted copy assignment.
   */
  MPCNetPolicyEvaluation& operator=(const MPCNetPolicyEvaluation&) = delete;

  /**
   * Run the policy evaluation.
   * @param [in] alpha: The mixture parameter for the behavioral controller.
   * @param [in] policyFilePath: The path to the file with the learned policy for the behavioral controller.
   * @param [in] timeStep: The time step for the forward simulation of the system with the behavioral controller.
   * @param [in] initialObservation: The initial system observation to start from (time and state required).
   * @param [in] modeSchedule: The mode schedule providing the event times and mode sequence.
   * @param [in] targetTrajectories: The target trajectories to be tracked.
   * @return The computed metrics.
   */
  metrics_t Run(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, const SystemObservation& initialObservation,
                const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories);
};

}  // namespace mpcnet
}  // namespace bolza
