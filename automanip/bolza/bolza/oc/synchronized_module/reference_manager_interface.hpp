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
 * @file reference_manager_interface.hpp
 * @brief Defines the interface for providing the ModeSchedule and the TargetTrajectories to the solver.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/reference/mode_schedule.hpp>
#include <bolza/core/reference/target_trajectories.hpp>

namespace bolza {

/**
 * Defines the interface for providing the ModeSchedule and the TargetTrajectories to the solver.
 */
class ReferenceManagerInterface {
 public:
  /** Constructor */
  ReferenceManagerInterface() = default;

  /** Destructor */
  virtual ~ReferenceManagerInterface() = default;

  /** Disable copy / move */
  ReferenceManagerInterface& operator=(const ReferenceManagerInterface&) = delete;
  /**
   * @brief Constructs a ReferenceManagerInterface.
   */
  ReferenceManagerInterface(const ReferenceManagerInterface&) = delete;
  ReferenceManagerInterface& operator=(ReferenceManagerInterface&&) = delete;
  /**
   * @brief Constructs a ReferenceManagerInterface.
   */
  ReferenceManagerInterface(ReferenceManagerInterface&&) = delete;

  /**
   * The method is called right before the solver runs and before any other SolverSynchronizedModule::PreSolverRun().
   *
   * @param [in] initTime: Start time of the optimization horizon.
   * @param [in] finalTime: Final time of the optimization horizon.
   * @param [in] initState: State at the start of the optimization horizon.
   */
  virtual void PreSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState){};

  /**
   * @brief Returns a const reference to the active ModeSchedule.
   * @return Result value.
   */
  virtual const ModeSchedule& GetModeSchedule() const = 0;

  /**
   * @brief Sets the ModeSchedule to the buffer. The buffer will move to active ModeSchedule once PreSolverRun() is called.
   * @note: This method must be thread safe.
   * @param [in] modeSchedule: Mode Schedule.
   */
  virtual void SetModeSchedule(const ModeSchedule& modeSchedule) = 0;

  /**
   * @brief Move the ModeSchedule to the buffer. The buffer will move to active ModeSchedule once PreSolverRun() is called.
   * @note: This method must be thread safe.
   * @param [in,out] modeSchedule: Mode Schedule.
   */
  virtual void SetModeSchedule(ModeSchedule&& modeSchedule) = 0;

  /**
   * @brief Returns a const reference to the active TargetTrajectories.
   * @return Result value.
   */
  virtual const TargetTrajectories& GetTargetTrajectories() const = 0;

  /**
   * @brief Sets the TargetTrajectories to the buffer. The buffer will move to active ModeSchedule once PreSolverRun() is called.
   * @note: This method must be thread safe.
   * @param [in] targetTrajectories: Target Trajectories.
   */
  virtual void SetTargetTrajectories(const TargetTrajectories& targetTrajectories) = 0;

  /**
   * @brief Move the TargetTrajectories to the buffer. The buffer will move to active ModeSchedule once PreSolverRun() is called.
   * @note: This method must be thread safe.
   * @param [in,out] targetTrajectories: Target Trajectories.
   */
  virtual void SetTargetTrajectories(TargetTrajectories&& targetTrajectories) = 0;
};

}  // namespace bolza
