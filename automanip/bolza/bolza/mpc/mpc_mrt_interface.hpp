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
 * @file mpc_mrt_interface.hpp
 * @brief A lean ROS independent interface to OCS2. In incorporates the functionality of the MPC and the MRT (trajectory tracking) modules.
 */


#include <condition_variable>
#include <csignal>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include <bolza/core/misc/benchmark.hpp>
#include <bolza/core/model_data/multiplier.hpp>
#include "bolza/mpc/mpc_base.hpp"
#include "bolza/mpc/mrt_base.hpp"

namespace bolza {

/**
 * A lean ROS independent interface to OCS2. In incorporates the functionality of the MPC and the MRT (trajectory tracking) modules.
 * Please refer to ocs2_double_integrator_example for a minimal example and tests
 */
class MPCMRTInterface final : public MRTBase {
 public:
  /**
   * Constructor
   * @param [in] mpc: The underlying MPC class to be used.
   */
  explicit MPCMRTInterface(MPCBase& mpc);

  /**
   * @brief Destructor.
   */
  ~MPCMRTInterface() override = default;

  /**
   * @brief Reset Mpc Node.
   * @param [in] initTargetTrajectories: Init Target Trajectories.
   */
  void ResetMpcNode(const TargetTrajectories& initTargetTrajectories) override;

  /**
   * @brief Set Current Observation.
   * @param [in] currentObservation: Current Observation.
   */
  void SetCurrentObservation(const SystemObservation& currentObservation) override;

  /*
   * Gets the ReferenceManager which manages both ModeSchedule and TargetTrajectories.
   */
  /**
   * @brief Get Reference Manager.
   * @return Result value.
   */
  ReferenceManagerInterface& GetReferenceManager();
  /**
   * @brief Get Reference Manager.
   * @return Result value.
   */
  const ReferenceManagerInterface& GetReferenceManager() const;

  /**
   * Advance the mpc module for one iteration. The evaluation methods can be called while this method is running. They will evaluate the
   * control law that was up-to-date at the last UpdatePolicy() call.
   */
  void AdvanceMpc();

  /**
   * @brief Retrieves the gain matrix from solver capable of optimizing over LinearController type.
   *
   * @note This method is not thread-safe, meaning you can only access this data safely at the end of each MPC iteration.
   *
   * @param [in] time: query time
   * @return The gain matrix.
   */
  matrix_t GetLinearFeedbackGain(scalar_t time);

  /**
   * @brief Access the solver's internal value function.
   *
   * @note This method is not thread-safe, meaning you can only access this data safely at the end of each MPC iteration.
   *
   * @param [in] time: query time
   * @param [in] state: query state
   * @return The quadratic approximation of the value function at the requested time and state.
   */
  ScalarFunctionQuadraticApproximation GetValueFunction(scalar_t time, const vector_t& state) const;

  /**
   * @brief Computes the Lagrange multiplier related to the state-input constraints
   *
   * @note This method is not thread-safe, meaning you can only access this data safely at the end of each MPC iteration.
   *
   * @param [in] time: query time
   * @param [in] state: query state
   * @return The Lagrange multiplier at the requested time and state
   */
  vector_t GetStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const;

  /**
   * @brief Returns the intermediate dual solution at the given time.
   *
   * @note This method is not thread-safe, meaning you can only access this data safely at the end of each MPC iteration.
   *
   * @param [in] time: The inquiry time
   * @return The collection of multipliers associated to state/state-input, equality/inequality Lagrangian terms.
   */
  MultiplierCollection GetIntermediateDualSolution(scalar_t time) const;

 private:
  /**
   * Updates the buffer variables from the MPC object. This method is automatically called by AdvanceMpc()
   *
   * @param [in] mpcInitObservation: The observation used to run the MPC.
   */
  void CopyToBuffer(const SystemObservation& mpcInitObservation);

  MPCBase& mpc_;
  benchmark::RepeatedTimer mpcTimer_;

  // MPC inputs
  SystemObservation currentObservation_;
  std::mutex observationMutex_;
};

}  // namespace bolza
