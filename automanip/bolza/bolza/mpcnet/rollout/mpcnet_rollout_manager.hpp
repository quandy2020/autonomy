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
 * @file mpcnet_rollout_manager.hpp
 * @brief A class to manage the data generation and policy evaluation rollouts for MPC-Net.
 */


#include <bolza/core/thread_support/thread_pool.hpp>

#include "bolza/mpcnet/rollout/mpcnet_data_generation.hpp"
#include "bolza/mpcnet/rollout/mpcnet_policy_evaluation.hpp"

namespace bolza {
namespace mpcnet {

/**
 *  A class to manage the data generation and policy evaluation rollouts for MPC-Net.
 */
class MPCNetRolloutManager {
 public:
  /**
   * Constructor.
   * @note The first nDataGenerationThreads pointers will be used for the data generation and the next nPolicyEvaluationThreads pointers for
   * the policy evaluation.
   * @param [in] nDataGenerationThreads: Number of data generation threads.
   * @param [in] nPolicyEvaluationThreads: Number of policy evaluation threads.
   * @param [in] mpcPtrs: Pointers to the MPC solvers to be used.
   * @param [in] mpcnetPtrs: Pointers to the MPC-Net policies to be used.
   * @param [in] rolloutPtrs: Pointers to the rollouts to be used.
   * @param [in] mpcnetDefinitionPtrs: Pointers to the MPC-Net definitions to be used.
   * @param [in] referenceManagerPtrs: Pointers to the reference managers to be used.
   */
  MPCNetRolloutManager(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads, std::vector<std::unique_ptr<MPCBase>> mpcPtrs,
                       std::vector<std::unique_ptr<MPCNetControllerBase>> mpcnetPtrs, std::vector<std::unique_ptr<RolloutBase>> rolloutPtrs,
                       std::vector<std::shared_ptr<MPCNetDefinitionBase>> mpcnetDefinitionPtrs,
                       std::vector<std::shared_ptr<ReferenceManagerInterface>> referenceManagerPtrs);

  /**
   * Default destructor.
   */
  virtual ~MPCNetRolloutManager() = default;

  /**
   * Starts the data genration forward simulated by a behavioral controller.
   * @param [in] alpha: The mixture parameter for the behavioral controller.
   * @param [in] policyFilePath: The path to the file with the learned policy for the behavioral controller.
   * @param [in] timeStep: The time step for the forward simulation of the system with the behavioral controller.
   * @param [in] dataDecimation: The integer factor used for downsampling the data signal.
   * @param [in] nSamples: The number of samples drawn from a multivariate normal distribution around the nominal states.
   * @param [in] samplingCovariance: The covariance matrix used for sampling from a multivariate normal distribution.
   * @param [in] initialObservations: The initial system observations to start from (time and state required).
   * @param [in] modeSchedules: The mode schedules providing the event times and mode sequence.
   * @param [in] targetTrajectories: The target trajectories to be tracked.
   */
  void StartDataGeneration(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation, size_t nSamples,
                           const matrix_t& samplingCovariance, const std::vector<SystemObservation>& initialObservations,
                           const std::vector<ModeSchedule>& modeSchedules, const std::vector<TargetTrajectories>& targetTrajectories);

  /**
   * Check if data generation is done.
   * @return True if done.
   */
  bool IsDataGenerationDone();

  /**
   * Get the data generated from the data generation rollout.
   * @return The generated data.
   */
  const data_array_t& GetGeneratedData();

  /**
   * Starts the policy evaluation forward simulated by a behavioral controller.
   * @param [in] alpha: The mixture parameter for the behavioral controller.
   * @param [in] policyFilePath: The path to the file with the learned policy for the behavioral controller.
   * @param [in] timeStep: The time step for the forward simulation of the system with the behavioral controller.
   * @param [in] initialObservations: The initial system observations to start from (time and state required).
   * @param [in] modeSchedules: The mode schedules providing the event times and mode sequence.
   * @param [in] targetTrajectories: The target trajectories to be tracked.
   */
  void StartPolicyEvaluation(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                             const std::vector<SystemObservation>& initialObservations, const std::vector<ModeSchedule>& modeSchedules,
                             const std::vector<TargetTrajectories>& targetTrajectories);

  /**
   * Check if policy evaluation is done.
   * @return True if done.
   */
  bool IsPolicyEvaluationDone();

  /**
   * Get the metrics computed from the policy evaluation rollout.
   * @return The computed metrics.
   */
  metrics_array_t GetComputedMetrics();

 private:
  // data generation variables
  size_t nDataGenerationThreads_;
  std::atomic_int nDataGenerationTasksDone_;
  std::unique_ptr<ThreadPool> dataGenerationThreadPoolPtr_;
  std::vector<std::unique_ptr<MPCNetDataGeneration>> dataGenerationPtrs_;
  std::vector<std::future<const data_array_t*>> dataGenerationFtrs_;
  data_array_t dataArray_;
  // policy evaluation variables
  size_t nPolicyEvaluationThreads_;
  std::atomic_int nPolicyEvaluationTasksDone_;
  std::unique_ptr<ThreadPool> policyEvaluationThreadPoolPtr_;
  std::vector<std::unique_ptr<MPCNetPolicyEvaluation>> policyEvaluationPtrs_;
  std::vector<std::future<metrics_t>> policyEvaluationFtrs_;
};

}  // namespace mpcnet
}  // namespace bolza
