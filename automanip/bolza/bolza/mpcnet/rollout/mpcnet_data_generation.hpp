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
 * @file mpcnet_data_generation.hpp
 * @brief A class for generating data from a system that is forward simulated with a behavioral controller.
 */


#include "bolza/mpcnet/rollout/mpcnet_data.hpp"
#include "bolza/mpcnet/rollout/mpcnet_rollout_base.hpp"

namespace bolza {
namespace mpcnet {

/**
 *  A class for generating data from a system that is forward simulated with a behavioral controller.
 *  @note Usually the behavioral controller moves from the MPC policy to the MPC-Net policy throughout the training process.
 */
class MPCNetDataGeneration final : public MPCNetRolloutBase {
 public:
  /**
   * Constructor.
   * @param [in] mpcPtr: Pointer to the MPC solver to be used (this class takes ownership).
   * @param [in] mpcnetPtr: Pointer to the MPC-Net policy to be used (this class takes ownership).
   * @param [in] rolloutPtr: Pointer to the rollout to be used (this class takes ownership).
   * @param [in] mpcnetDefinitionPtr: Pointer to the MPC-Net definitions to be used (shared ownership).
   * @param [in] referenceManagerPtr: Pointer to the reference manager to be used (shared ownership).
   */
  MPCNetDataGeneration(std::unique_ptr<MPCBase> mpcPtr, std::unique_ptr<MPCNetControllerBase> mpcnetPtr,
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
  ~MPCNetDataGeneration() override = default;

  /**
   * Deleted copy constructor.
   */
  MPCNetDataGeneration(const MPCNetDataGeneration&) = delete;

  /**
   * Deleted copy assignment.
   */
  MPCNetDataGeneration& operator=(const MPCNetDataGeneration&) = delete;

  /**
   * Run the data generation.
   * @param [in] alpha: The mixture parameter for the behavioral controller.
   * @param [in] policyFilePath: The path to the file with the learned policy for the behavioral controller.
   * @param [in] timeStep: The time step for the forward simulation of the system with the behavioral controller.
   * @param [in] dataDecimation: The integer factor used for downsampling the data signal.
   * @param [in] nSamples: The number of samples drawn from a multivariate normal distribution around the nominal states.
   * @param [in] samplingCovariance: The covariance matrix used for sampling from a multivariate normal distribution.
   * @param [in] initialObservation: The initial system observation to start from (time and state required).
   * @param [in] modeSchedule: The mode schedule providing the event times and mode sequence.
   * @param [in] targetTrajectories: The target trajectories to be tracked.
   * @return Pointer to the data array with the generated data.
   */
  const data_array_t* Run(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation, size_t nSamples,
                          const matrix_t& samplingCovariance, const SystemObservation& initialObservation, const ModeSchedule& modeSchedule,
                          const TargetTrajectories& targetTrajectories);

 private:
  data_array_t dataArray_;
};

}  // namespace mpcnet
}  // namespace bolza
