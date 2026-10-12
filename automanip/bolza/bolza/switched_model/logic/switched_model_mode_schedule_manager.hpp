#pragma once

/**
 * @file switched_model_mode_schedule_manager.hpp
 * @brief Manages the Target Trajectories and ModeSchedule for switched model.
 */


#include <bolza/core/thread_support/synchronized.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>

#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/dynamics/com_kino_dynamics_parameters.hpp"
#include "bolza/switched_model/foot_planner/swing_trajectory_planner.hpp"
#include "bolza/switched_model/logic/gait_schedule.hpp"
#include "bolza/switched_model/terrain/terrain_plane.hpp"

namespace switched_model {

/**
 * Manages the Target Trajectories and ModeSchedule for switched model.
 */
class SwitchedModelModeScheduleManager : public bolza::ReferenceManager {
 public:
  /**
   * @brief Constructs a SwitchedModelModeScheduleManager.
   * @param [in] gaitSchedule: Gait Schedule.
   * @param [in] swingTrajectory: Swing Trajectory.
   * @param [in] terrainModel: Terrain Model.
   */
  SwitchedModelModeScheduleManager(std::unique_ptr<GaitSchedule> gaitSchedule, std::unique_ptr<SwingTrajectoryPlanner> swingTrajectory,
                                   std::unique_ptr<TerrainModel> terrainModel);

  /**
   * @brief Destructor.
   */
  ~SwitchedModelModeScheduleManager() override = default;

  /**
   * @brief Get Contact Flags.
   * @param [in] time: Time.
   * @return Result value.
   */
  contact_flag_t GetContactFlags(scalar_t time) const;

  /**
   * @brief Get Gait Schedule.
   * @return Result value.
   */
  bolza::Synchronized<GaitSchedule>& GetGaitSchedule() { return gaitSchedule_; }
  /**
   * @brief Get Gait Schedule.
   * @return Result value.
   */
  const bolza::Synchronized<GaitSchedule>& GetGaitSchedule() const { return gaitSchedule_; }

  /**
   * @brief Get Swing Trajectory Planner.
   * @return Result value.
   */
  const SwingTrajectoryPlanner& GetSwingTrajectoryPlanner() const { return *swingTrajectoryPtr_; }

  /**
   * @brief Get Terrain Model.
   * @return Result value.
   */
  bolza::Synchronized<TerrainModel>& GetTerrainModel() { return terrainModel_; }
  /**
   * @brief Get Terrain Model.
   * @return Result value.
   */
  const bolza::Synchronized<TerrainModel>& GetTerrainModel() const { return terrainModel_; }

 private:
  /**
   * @brief Modify References.
   * @param [in] initTime: Init Time.
   * @param [in] finalTime: Final Time.
   * @param [in] initState: Init State.
   * @param [in,out] targetTrajectories: Target Trajectories.
   * @param [in,out] modeSchedule: Mode Schedule.
   */
  void ModifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, bolza::TargetTrajectories& targetTrajectories,
                        bolza::ModeSchedule& modeSchedule) override;

  bolza::Synchronized<GaitSchedule> gaitSchedule_;
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
  bolza::Synchronized<TerrainModel> terrainModel_;
};

}  // namespace switched_model
