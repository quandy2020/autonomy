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
 * @file mrt_base.hpp
 * @brief This class implements core MRT (Model Reference Tracking) functionality.
 */


#include <Eigen/Dense>

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>

#include <bolza/core/types.hpp>
#include <bolza/core/control/controller_base.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/core/reference/mode_schedule.hpp>
#include <bolza/core/reference/target_trajectories.hpp>
#include <bolza/oc/oc_data/performance_index.hpp>
#include <bolza/oc/oc_data/primal_solution.hpp>
#include <bolza/oc/rollout/rollout_base.hpp>

#include "bolza/mpc/command_data.hpp"
#include "bolza/mpc/mrt_observer.hpp"
#include "bolza/mpc/system_observation.hpp"

namespace bolza {

/**
 * This class implements core MRT (Model Reference Tracking) functionality.
 * The responsibility of filling the buffer variables is left to the deriving classes.
 */
class MRTBase {
 public:
  /**
   * @brief Constructs a MRTBase.
   */
  MRTBase();

  /**
   * @brief Destructor.
   */
  virtual ~MRTBase() = default;

  /**
   * Resets the class to its instantiated state.
   */
  void Reset();

  /**
   * Request the MPC node to reset. This method is a blocking method.
   *
   * @param [in] initTargetTrajectories: The initial desired cost trajectories.
   */
  virtual void ResetMpcNode(const TargetTrajectories& initTargetTrajectories) = 0;

  /**
   * @brief Whether the initial MPC policy has been already received.
   * @return True on success.
   */
  bool InitialPolicyReceived() const { return policyReceivedEver_; }

  /**
   * @brief setCurrentObservation notifies MPC of a new state
   * @param [in] observation: the current measurement to send to the MPC
   */
  virtual void SetCurrentObservation(const SystemObservation& observation) = 0;

  /**
   * Gets a reference to the command data corresponding to the current policy.
   * @warning access to the returned reference is not threadsafe. Read access and calls to UpdatePolicy() must be synced by the user.
   *
   * @return a constant reference to command data.
   */
  const CommandData& GetCommand() const;

  /**
   * Gets a reference to the performance indices data corresponding to the current policy.
   * @warning access to the returned reference is not threadsafe. Read access and calls to UpdatePolicy() must be synced by the user.
   *
   * @return a constant reference to performance indices data.
   */
  const PerformanceIndex& GetPerformanceIndices() const;

  /**
   * Gets a reference to current optimized policy.
   * @warning access to the returned reference is not threadsafe. Read access and calls to UpdatePolicy() must be synced by the user.
   *
   * @return constant reference to the policy data.
   */
  const PrimalSolution& GetPolicy() const;

  /**
   * @brief Initializes rollout class to roll out a feedback policy
   * @param [in] rolloutPtr: The rollout object to be used
   */
  void InitRollout(const RolloutBase* rolloutPtr);

  /**
   * @brief Evaluates the controller
   *
   * @param [in] currentTime: the query time.
   * @param [in] currentState: the query state.
   * @param [out] mpcState: the current nominal state of MPC.
   * @param [out] mpcInput: the optimized control input.
   * @param [out] mode: the active mode.
   */
  void EvaluatePolicy(scalar_t currentTime, const vector_t& currentState, vector_t& mpcState, vector_t& mpcInput, size_t& mode);

  /**
   * @brief Rolls out the control policy from the current time and state to get the next state and input using the MPC policy.
   *
   * @param [in] currentTime: start time of the rollout.
   * @param [in] currentState: state to start rollout from.
   * @param [in] timeStep: duration of the forward rollout.
   * @param [out] mpcState: the new forwarded state of MPC.
   * @param [out] mpcInput: the new control input of MPC.
   * @param [out] mode: the active mode.
   */
  void RolloutPolicy(scalar_t currentTime, const vector_t& currentState, const scalar_t& timeStep, vector_t& mpcState, vector_t& mpcInput,
                     size_t& mode);

  /**
   * Checks the data buffer for an update of the MPC policy. If a new policy
   * is available on the buffer this method will load it to the in-use policy.
   * This method also calls the ModifyActiveSolution() method.
   *
   * @return True if the policy is updated.
   */
  bool UpdatePolicy();

  /**
   * @brief rolloutSet: Whether or not the internal rollout object has been set
   * @return True if a rollout object is available.
   */
  bool IsRolloutSet() const { return rolloutPtr_ != nullptr; }

  /**
   * Adds an MRT observer to the policy update process
   */
  void AddMrtObserver(std::shared_ptr<MrtObserver> mrtObserver) { observerPtrArray_.push_back(std::move(mrtObserver)); };

 protected:
  /**
   * @brief Move To Buffer.
   * @param [in] commandDataPtr: Command Data Ptr.
   * @param [in] primalSolutionPtr: Primal Solution Ptr.
   * @param [in] performanceIndicesPtr: Performance Indices Ptr.
   */
  void MoveToBuffer(std::unique_ptr<CommandData> commandDataPtr, std::unique_ptr<PrimalSolution> primalSolutionPtr,
                    std::unique_ptr<PerformanceIndex> performanceIndicesPtr);

 private:
  /**
   * @brief Calls modifyActiveSolution on all mrt observers. This function is called while holding a policyBufferMutex lock.
   * @param [in] command: Command.
   * @param [in,out] primalSolution: Primal Solution.
   */
  void ModifyActiveSolution(const CommandData& command, PrimalSolution& primalSolution);

  /**
   * @brief Calls modifyBufferedSolution on all mrt observers. This function is called while holding a policyBufferMutex lock.
   * @param [in] commandBuffer: Command Buffer.
   * @param [in,out] primalSolutionBuffer: Primal Solution Buffer.
   */
  void ModifyBufferedSolution(const CommandData& commandBuffer, PrimalSolution& primalSolutionBuffer);

  // flags on state of the class
  std::atomic_bool policyReceivedEver_;
  bool newPolicyInBuffer_;  // whether a new policy is waiting to be swapped in

  // variables related to the MPC output
  std::unique_ptr<CommandData> activeCommandPtr_;
  std::unique_ptr<CommandData> bufferCommandPtr_;
  std::unique_ptr<PrimalSolution> activePrimalSolutionPtr_;
  std::unique_ptr<PrimalSolution> bufferPrimalSolutionPtr_;
  std::unique_ptr<PerformanceIndex> activePerformanceIndicesPtr_;
  std::unique_ptr<PerformanceIndex> bufferPerformanceIndicesPtr_;

  // thread safety
  mutable std::mutex bufferMutex_;  // for policy variables with the prefix (buffer*)
  const size_t mrtTrylockWarningThreshold_ = 5;
  size_t mrtTrylockWarningCount_;

  // variables needed for policy evaluation
  std::unique_ptr<RolloutBase> rolloutPtr_;

  std::vector<std::shared_ptr<MrtObserver>> observerPtrArray_;
};

}  // namespace bolza
