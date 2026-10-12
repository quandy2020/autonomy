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
 * @file reference_manager.hpp
 * @brief Implements the reference manager with a thread-safe buffer for setting and getting the references.
 */


#include "bolza/core/thread_support/buffered_value.hpp"
#include "bolza/oc/synchronized_module/reference_manager_interface.hpp"

namespace bolza {

/**
 * Implements the reference manager with a thread-safe buffer for setting and getting the references.
 * A protected virtual interface is provided to modify the references before each solver run.
 */
class ReferenceManager : public ReferenceManagerInterface {
 public:
  /**
   * @brief Constructs a ReferenceManager.
   * @param [in] initialTargetTrajectories: Initial Target Trajectories.
   * @param [in] initialModeSchedule: Initial Mode Schedule.
   */
  explicit ReferenceManager(TargetTrajectories initialTargetTrajectories = TargetTrajectories(),
                            ModeSchedule initialModeSchedule = ModeSchedule());

  /**
   * @brief Destructor.
   */
  ~ReferenceManager() override = default;

  /**
   * @brief Pre Solver Run.
   * @param [in] initTime: Init Time.
   * @param [in] finalTime: Final Time.
   * @param [in] initState: Init State.
   */
  void PreSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState) override;

  /**
   * @brief Get Mode Schedule.
   * @return Result value.
   */
  const ModeSchedule& GetModeSchedule() const override { return modeSchedule_.get(); }
  /**
   * @brief Set Mode Schedule.
   * @param [in] modeSchedule: Mode Schedule.
   */
  void SetModeSchedule(const ModeSchedule& modeSchedule) override { modeSchedule_.SetBuffer(modeSchedule); }
  /**
   * @brief Set Mode Schedule.
   */
  void SetModeSchedule(ModeSchedule&& modeSchedule) override { modeSchedule_.SetBuffer(std::move(modeSchedule)); }

  /**
   * @brief Get Target Trajectories.
   * @return Result value.
   */
  const TargetTrajectories& GetTargetTrajectories() const override { return targetTrajectories_.get(); }
  /**
   * @brief Set Target Trajectories.
   * @param [in] targetTrajectories: Target Trajectories.
   */
  void SetTargetTrajectories(const TargetTrajectories& targetTrajectories) override {
    /**
     * @brief Set Buffer.
     * @param [in] targetTrajectories: Target Trajectories.
     * @return Result value.
     */
    return targetTrajectories_.SetBuffer(targetTrajectories);
  }
  /**
   * @brief Set Target Trajectories.
   * @param [in,out] targetTrajectories: Target Trajectories.
   */
  void SetTargetTrajectories(TargetTrajectories&& targetTrajectories) override {
    /**
     * @brief Set Buffer.
     * @return Result value.
     */
    return targetTrajectories_.SetBuffer(std::move(targetTrajectories));
  }

 protected:
  /**
   * Modifies the active ModeSchedule and TargetTrajectories.
   *
   * @param [in] initTime: Start time of the optimization horizon.
   * @param [in] finalTime: Final time of the optimization horizon.
   * @param [in] initState: State at the start of the optimization horizon.
   * @param [in,out] targetTrajectories: The updated TargetTrajectories. If SetTargetTrajectories() has been called before,
   * TargetTrajectories is already updated by the set value.
   * @param [in,out] modeSchedule: The updated ModeSchedule. If SetModeSchedule() has been called before, modeSchedule is
   * already updated by the set value.
   */
  virtual void ModifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule) {}

 private:
  BufferedValue<ModeSchedule> modeSchedule_;
  BufferedValue<TargetTrajectories> targetTrajectories_;
};

}  // namespace bolza
