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
 * @file mpcnet_interface_base.hpp
 * @brief Base class for MPC-Net data-generation / evaluation interfaces.
 */


#include "bolza/mpcnet/rollout/mpcnet_rollout_manager.hpp"

namespace bolza {
namespace mpcnet {

/**
 * Base class for MPC-Net data-generation / evaluation interfaces.
 */
class MPCNetInterfaceBase {
 public:
  /**
   * Default destructor.
   */
  virtual ~MPCNetInterfaceBase() = default;

  /**
   * @brief Start Data Generation.
   * @see MPCNetRolloutManager::StartDataGeneration()
   * @param [in] alpha: Alpha.
   * @param [in] policyFilePath: Policy File Path.
   * @param [in] timeStep: Time Step.
   * @param [in] dataDecimation: Data Decimation.
   * @param [in] nSamples: N Samples.
   * @param [in] samplingCovariance: Sampling Covariance.
   * @param [in] initialObservations: Initial Observations.
   * @param [in] modeSchedules: Mode Schedules.
   * @param [in] targetTrajectories: Target Trajectories.
   */
  void StartDataGeneration(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation, size_t nSamples,
                           const matrix_t& samplingCovariance, const std::vector<SystemObservation>& initialObservations,
                           const std::vector<ModeSchedule>& modeSchedules, const std::vector<TargetTrajectories>& targetTrajectories);

  /**
   * @brief Is Data Generation Done.
   * @see MPCNetRolloutManager::IsDataGenerationDone()
   * @return True on success.
   */
  bool IsDataGenerationDone();

  /**
   * @brief Get Generated Data.
   * @see MPCNetRolloutManager::GetGeneratedData()
   * @return Result value.
   */
  data_array_t GetGeneratedData();

  /**
   * @brief Start Policy Evaluation.
   * @see MPCNetRolloutManager::StartPolicyEvaluation()
   * @param [in] alpha: Alpha.
   * @param [in] policyFilePath: Policy File Path.
   * @param [in] timeStep: Time Step.
   * @param [in] initialObservations: Initial Observations.
   * @param [in] modeSchedules: Mode Schedules.
   * @param [in] targetTrajectories: Target Trajectories.
   */
  void StartPolicyEvaluation(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                             const std::vector<SystemObservation>& initialObservations, const std::vector<ModeSchedule>& modeSchedules,
                             const std::vector<TargetTrajectories>& targetTrajectories);

  /**
   * @brief Is Policy Evaluation Done.
   * @see MPCNetRolloutManager::IsPolicyEvaluationDone()
   * @return True on success.
   */
  bool IsPolicyEvaluationDone();

  /**
   * @brief Get Computed Metrics.
   * @see MPCNetRolloutManager::GetComputedMetrics()
   * @return Result value.
   */
  metrics_array_t GetComputedMetrics();

 protected:
  /**
   * Default constructor.
   */
  MPCNetInterfaceBase() = default;

  std::unique_ptr<MPCNetRolloutManager> mpcnetRolloutManagerPtr_;
};

}  // namespace mpcnet
}  // namespace bolza
