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

#include <bolza/core/thread_support/synchronized.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>

#include "examples/legged_robot/foot_planner/swing_trajectory_planner.hpp"
#include "examples/legged_robot/gait/gait_schedule.hpp"
#include "examples/legged_robot/gait/motion_phase_definition.hpp"

namespace bolza {
namespace legged_robot {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

  ~SwitchedModelReferenceManager() override = default;

  void SetModeSchedule(const ModeSchedule& modeSchedule) override;

  contact_flag_t GetContactFlags(scalar_t time) const;

  const std::shared_ptr<GaitSchedule>& GetGaitSchedule() { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& GetSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

 private:
  void ModifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
  // Stance locks every foot. A base target then has no feasible step, and the
  // rollout diverges. Trot is selected only while the base is away from the goal.
  bool walking_ = false;
};

}  // namespace legged_robot
}  // namespace bolza
