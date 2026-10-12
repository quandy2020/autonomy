//
// Created by rgrandia on 18.03.20.
//

#include "bolza/switched_model/logic/switched_model_mode_schedule_manager.hpp"

#include "bolza/switched_model/core/motion_phase_definition.hpp"

namespace switched_model {

SwitchedModelModeScheduleManager::SwitchedModelModeScheduleManager(std::unique_ptr<GaitSchedule> gaitSchedule,
                                                                   std::unique_ptr<SwingTrajectoryPlanner> swingTrajectory,
                                                                   std::unique_ptr<TerrainModel> terrainModel)
    : gaitSchedule_(std::move(gaitSchedule)), swingTrajectoryPtr_(std::move(swingTrajectory)), terrainModel_(std::move(terrainModel)) {}

contact_flag_t SwitchedModelModeScheduleManager::GetContactFlags(scalar_t time) const {
  return ModeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

void SwitchedModelModeScheduleManager::ModifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                        bolza::TargetTrajectories& targetTrajectories, bolza::ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  {
    auto lockedGaitSchedulePtr = gaitSchedule_.lock();
    lockedGaitSchedulePtr->advanceToTime(initTime);
    modeSchedule = lockedGaitSchedulePtr->getModeSchedule(timeHorizon + swingTrajectoryPtr_->settings().referenceExtensionAfterHorizon);
  }

  // Transfer terrain ownership if a new terrain is available
  std::unique_ptr<TerrainModel> newTerrain;
  terrainModel_.swap(newTerrain);  // Thread-safe swap with lockable Terrain
  if (newTerrain) {
    swingTrajectoryPtr_->updateTerrain(std::move(newTerrain));
  }

  // Prepare swing motions
  swingTrajectoryPtr_->updateSwingMotions(initTime, finalTime, initState, targetTrajectories, modeSchedule);
}

}  // namespace switched_model
