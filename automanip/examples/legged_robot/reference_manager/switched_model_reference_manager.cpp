/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "examples/legged_robot/reference_manager/switched_model_reference_manager.hpp"

#include <cmath>
#include <iostream>

namespace automanip {
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
void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule) {
  ReferenceManager::setModeSchedule(modeSchedule);
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  if (initState.size() >= 12 && !targetTrajectories.empty()) {
    const vector_t goal = targetTrajectories.getDesiredState(finalTime);
    if (goal.size() >= 12) {
      const double dx = goal(6) - initState(6);
      const double dy = goal(7) - initState(7);
      const double dyaw = std::atan2(std::sin(goal(9) - initState(9)), std::cos(goal(9) - initState(9)));
      const double distance = std::sqrt(dx * dx + dy * dy);
      const bool want_walk = walking_ ? (distance > 0.05 || std::abs(dyaw) > 0.08)
                                      : (distance > 0.12 || std::abs(dyaw) > 0.2);
      if (want_walk != walking_) {
        walking_ = want_walk;
        std::cerr << "[SwitchedModelReferenceManager] gait " << (walking_ ? "trot" : "stance") << std::endl;
        gaitSchedulePtr_->insertModeSequenceTemplate(walking_ ? TrotGait() : StanceGait(), initTime,
                                                     finalTime + timeHorizon);
      }
    }
  }
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  const scalar_t terrainHeight = 0.0;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}

}  // namespace legged_robot
}  // namespace automanip
