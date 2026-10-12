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

#include "examples/legged_robot/reference_manager/switched_model_reference_manager.hpp"

#include <cmath>
#include <iostream>

namespace bolza {
namespace legged_robot {
namespace {

ModeSequenceTemplate TrotGait() {
  // Diagonal pairs, 0.35 s each. Stance feet keep zero velocity on the ground.
  return ModeSequenceTemplate({0.0, 0.35, 0.70}, {ModeNumber::LF_RH, ModeNumber::RF_LH});
}

ModeSequenceTemplate StanceGait() {
  return ModeSequenceTemplate({0.0, 0.5}, {ModeNumber::STANCE});
}

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::SetModeSchedule(const ModeSchedule& modeSchedule) {
  ReferenceManager::SetModeSchedule(modeSchedule);
  gaitSchedulePtr_->SetModeSchedule(modeSchedule);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::GetContactFlags(scalar_t time) const {
  return ModeNumber2StanceLeg(this->GetModeSchedule().ModeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::ModifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  bool switch_gait = false;
  if (initState.size() >= 12 && !targetTrajectories.empty()) {
    const vector_t goal = targetTrajectories.GetDesiredState(finalTime);
    if (goal.size() >= 12) {
      const double dx = goal(6) - initState(6);
      const double dy = goal(7) - initState(7);
      const double dyaw = std::atan2(std::sin(goal(9) - initState(9)), std::cos(goal(9) - initState(9)));
      const double distance = std::sqrt(dx * dx + dy * dy);
      const bool want_walk = walking_ ? (distance > 0.05 || std::abs(dyaw) > 0.08)
                                      : (distance > 0.12 || std::abs(dyaw) > 0.2);
      if (want_walk != walking_) {
        walking_ = want_walk;
        switch_gait = true;
        std::cerr << "[SwitchedModelReferenceManager] gait " << (walking_ ? "trot" : "stance") << std::endl;
      }
    }
  }
  modeSchedule = gaitSchedulePtr_->GetModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  const scalar_t terrainHeight = 0.0;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);

  // GaitReceiver inserts the template at finalTime, after this horizon has been
  // built. Replacing the modes inside [initTime, finalTime] makes trajectory
  // spreading walk off the mode array (bad_alloc / process abort).
  if (switch_gait) {
    gaitSchedulePtr_->insertModeSequenceTemplate(walking_ ? TrotGait() : StanceGait(), finalTime, timeHorizon);
  }
}

}  // namespace legged_robot
}  // namespace bolza
