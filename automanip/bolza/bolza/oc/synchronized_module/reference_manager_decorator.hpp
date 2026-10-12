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
 * @file reference_manager_decorator.hpp
 * @brief Implements the basic decorator functionality for the ReferenceManager.
 */


#include "bolza/oc/synchronized_module/reference_manager_interface.hpp"

namespace bolza {

/**
 * Implements the basic decorator functionality for the ReferenceManager
 * Forwards all methods to the wrapped object.
 */
class ReferenceManagerDecorator : public ReferenceManagerInterface {
 public:
  explicit ReferenceManagerDecorator(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
      /**
       * @brief Reference Manager Ptr.
       * @return Result value.
       */
      : referenceManagerPtr_(std::move(referenceManagerPtr)) {
    if (referenceManagerPtr_ == nullptr) {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("[ReferenceManagerDecorator] ReferenceManager pointer is nullptr!");
    }
  }

  /**
   * @brief Destructor.
   */
  ~ReferenceManagerDecorator() override = default;

  /**
   * @brief Pre Solver Run.
   * @param [in] initTime: Init Time.
   * @param [in] finalTime: Final Time.
   * @param [in] initState: Init State.
   */
  void PreSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState) override {
    /**
     * @brief Pre Solver Run.
     * @param [in] initTime: Init Time.
     * @param [in] finalTime: Final Time.
     * @param [in] initState: Init State.
     * @return Result value.
     */
    referenceManagerPtr_->PreSolverRun(initTime, finalTime, initState);
  }

  /**
   * @brief Get Mode Schedule.
   * @return Result value.
   */
  const ModeSchedule& GetModeSchedule() const override { return referenceManagerPtr_->GetModeSchedule(); }
  /**
   * @brief Set Mode Schedule.
   * @param [in] modeSchedule: Mode Schedule.
   */
  void SetModeSchedule(const ModeSchedule& modeSchedule) override { referenceManagerPtr_->SetModeSchedule(modeSchedule); }
  /**
   * @brief Set Mode Schedule.
   */
  void SetModeSchedule(ModeSchedule&& modeSchedule) override { referenceManagerPtr_->SetModeSchedule(std::move(modeSchedule)); }

  /**
   * @brief Get Target Trajectories.
   * @return Result value.
   */
  const TargetTrajectories& GetTargetTrajectories() const override { return referenceManagerPtr_->GetTargetTrajectories(); }
  /**
   * @brief Set Target Trajectories.
   * @param [in] targetTrajectories: Target Trajectories.
   */
  void SetTargetTrajectories(const TargetTrajectories& targetTrajectories) override {
    /**
     * @brief Set Target Trajectories.
     * @param [in] targetTrajectories: Target Trajectories.
     * @return Result value.
     */
    referenceManagerPtr_->SetTargetTrajectories(targetTrajectories);
  }
  /**
   * @brief Set Target Trajectories.
   * @param [in,out] targetTrajectories: Target Trajectories.
   */
  void SetTargetTrajectories(TargetTrajectories&& targetTrajectories) override {
    /**
     * @brief Set Target Trajectories.
     * @return Result value.
     */
    referenceManagerPtr_->SetTargetTrajectories(std::move(targetTrajectories));
  }

 protected:
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;
};
}  // namespace bolza
