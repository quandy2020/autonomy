//
// Created by rgrandia on 13.03.20.
//

#pragma once

/**
 * @file swing_trajectory_planner.hpp
 * @brief swing trajectory planner (bolza).
 */


#include <bolza/core/types.hpp>
#include <bolza/core/reference/target_trajectories.hpp>

#include "bolza/switched_model/core/inverse_kinematics_model_base.hpp"
#include "bolza/switched_model/core/kinematics_model_base.hpp"
#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/foot_planner/foot_phase.hpp"
#include "bolza/switched_model/logic/single_leg_logic.hpp"
#include "bolza/switched_model/terrain/convex_terrain.hpp"
#include "bolza/switched_model/terrain/terrain_model.hpp"
#include "bolza/switched_model/terrain/terrain_plane.hpp"

namespace switched_model {

struct SwingTrajectoryPlannerSettings {
  scalar_t liftOffVelocity = 0.0;
  scalar_t touchDownVelocity = 0.0;
  scalar_t swingHeight = 0.1;

  scalar_t errorGain = 0.0;          // proportional gain for returning to the planned swing trajectory. 10-90%-rise_time ~= 2.2 / errorGain
                                     // alternatively can be measured as (velocity feedback) / (tracking error) ([m/s] / [m])
  scalar_t swingTimeScale = 0.15;    // swing phases shorter than this time will be scaled down in height and velocity
  scalar_t sdfMidswingMargin = 0.0;  // desired sdf based clearance in the middle of the swing phase [m]
  scalar_t terrainMargin = 0.0;      // shrinkage of the convex terrain constrains in [m]

  scalar_t previousFootholdFactor = 0.0;        // factor in [0, 1] with which to take previous foothold into account.
  scalar_t previousFootholdDeadzone = 0.0;      // previous foothold is taken if the new reference is within this threshold. [m]
  scalar_t previousFootholdTimeDeadzone = 0.0;  // previous foothold is taken if the contact phase is starting withing this time. [s]
  scalar_t invertedPendulumHeight = 0.5;        // height used for the inverted pendulum foothold adjustment

  scalar_t nominalLegExtension = 0.55;     // Leg extension beyond this length [m] will be penalized in terrain selection
  scalar_t legOverExtensionPenalty = 5.0;  // Weight of the leg overextension penalty

  scalar_t referenceExtensionAfterHorizon = 1.0;  // base and foot references generated for this amount of seconds after the horizon ends.
  scalar_t maximumReferenceSampleTime = 0.05;     // if the reference trajectory has samples with longer intervals, it will be subsampled.

  bool swingTrajectoryFromReference = false;  // Flag to take the swing trajectory from the reference trajectory
};

/**
 * @brief Load Swing Trajectory Settings.
 * @param [in] filename: Filename.
 * @param [in] verbose: Verbose.
 * @return Result value.
 */
SwingTrajectoryPlannerSettings LoadSwingTrajectorySettings(const std::string& filename, bool verbose = true);

using inverse_kinematics_function_t = std::function<vector3_t(int, const vector3_t&)>;

class SwingTrajectoryPlanner {
 public:
  /**
   * @brief Constructs a SwingTrajectoryPlanner.
   * @param [in] settings: Settings.
   * @param [in] kinematicsModel: Kinematics Model.
   * @param [in] inverseKinematicsModelPtr: Inverse Kinematics Model Ptr.
   */
  SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const KinematicsModelBase<scalar_t>& kinematicsModel,
                         const InverseKinematicsModelBase* inverseKinematicsModelPtr);

  // Update terrain model
  /**
   * @brief Update Terrain.
   * @param [in] terrainModel: Terrain Model.
   */
  void UpdateTerrain(std::unique_ptr<TerrainModel> terrainModel);

  // Access the SDF of the current terrain model
  /**
   * @brief Get Signed Distance Field.
   * @return Pointer; may be null.
   */
  const SignedDistanceField* GetSignedDistanceField() const;

  // Main interface preparing all swing trajectories in cartesian space (called by reference manager)
  /**
   * @brief Update Swing Motions.
   * @param [in] initTime: Init Time.
   * @param [in] finalTime: Final Time.
   * @param [in] currentState: Current State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] modeSchedule: Mode Schedule.
   */
  void UpdateSwingMotions(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState,
                          const bolza::TargetTrajectories& targetTrajectories, const bolza::ModeSchedule& modeSchedule);

  // Main access method for the generated cartesian references.
  /**
   * @brief Get Foot Phase.
   * @param [in] leg: Leg.
   * @param [in] time: Time.
   * @return Result value.
   */
  const FootPhase& GetFootPhase(size_t leg, scalar_t time) const;

  // Accessed by precomputation to generate the motion reference, used in the controller to visualize the generated references
  /**
   * @brief Get Target Trajectories.
   * @return Result value.
   */
  const bolza::TargetTrajectories& GetTargetTrajectories() const { return targetTrajectories_; }

  // Accessed by the controller for visualization
  /**
   * @brief Get Nominal Footholds.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  std::vector<ConvexTerrain> GetNominalFootholds(size_t leg) const { return nominalFootholdsPerLeg_[leg]; }

  // Accessed by the controller for visualization
  /**
   * @brief Get Heuristic Footholds.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  std::vector<vector3_t> GetHeuristicFootholds(size_t leg) const { return heuristicFootholdsPerLeg_[leg]; }

  // Read settings
  /**
   * @brief Settings.
   * @return Result value.
   */
  const SwingTrajectoryPlannerSettings& Settings() const { return settings_; }

 private:
  /**
   * @brief Update Last Contact.
   * @param [in] leg: Leg.
   * @param [in] expectedLiftOff: Expected Lift Off.
   * @param [in] currentFootPosition: Current Foot Position.
   * @param [in] terrainModel: Terrain Model.
   */
  void UpdateLastContact(int leg, scalar_t expectedLiftOff, const vector3_t& currentFootPosition, const TerrainModel& terrainModel);

  /**
   * @brief Generate Swing Trajectories.
   * @param [in] leg: Leg.
   * @param [in] contactTimings: Contact Timings.
   * @param [in] finalTime: Final Time.
   * @return Owning pointer.
   */
  std::pair<std::vector<scalar_t>, std::vector<std::unique_ptr<FootPhase>>> GenerateSwingTrajectories(
      int leg, const std::vector<ContactTiming>& contactTimings, scalar_t finalTime) const;

  /**
   * @brief Extract Swing Trajectories From Reference.
   * @param [in] leg: Leg.
   * @param [in] contactTimings: Contact Timings.
   * @param [in] finalTime: Final Time.
   * @return Owning pointer.
   */
  std::pair<std::vector<scalar_t>, std::vector<std::unique_ptr<FootPhase>>> ExtractSwingTrajectoriesFromReference(
      int leg, const std::vector<ContactTiming>& contactTimings, scalar_t finalTime) const;

  /**
   * @brief Select Heuristic Footholds.
   * @param [in] leg: Leg.
   * @param [in] contactTimings: Contact Timings.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] initTime: Init Time.
   * @param [in] currentState: Current State.
   * @param [in] finalTime: Final Time.
   * @return Result value.
   */
  std::vector<vector3_t> SelectHeuristicFootholds(int leg, const std::vector<ContactTiming>& contactTimings,
                                                  const bolza::TargetTrajectories& targetTrajectories, scalar_t initTime,
                                                  const comkino_state_t& currentState, scalar_t finalTime) const;

  /**
   * @brief Select Nominal Foothold Terrain.
   * @param [in] leg: Leg.
   * @param [in] contactTimings: Contact Timings.
   * @param [in] heuristicFootholds: Heuristic Footholds.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] initTime: Init Time.
   * @param [in] currentState: Current State.
   * @param [in] finalTime: Final Time.
   * @param [in] terrainModel: Terrain Model.
   * @return Result value.
   */
  std::vector<ConvexTerrain> SelectNominalFootholdTerrain(int leg, const std::vector<ContactTiming>& contactTimings,
                                                          const std::vector<vector3_t>& heuristicFootholds,
                                                          const bolza::TargetTrajectories& targetTrajectories, scalar_t initTime,
                                                          const comkino_state_t& currentState, scalar_t finalTime,
                                                          const TerrainModel& terrainModel) const;

  /**
   * @brief Apply Swing Motion Scaling.
   * @param [in,out] liftOff: Lift Off.
   * @param [in,out] touchDown: Touch Down.
   * @param [in,out] swingProfile: Swing Profile.
   */
  void ApplySwingMotionScaling(SwingPhase::SwingEvent& liftOff, SwingPhase::SwingEvent& touchDown,
                               SwingPhase::SwingProfile& swingProfile) const;

  /**
   * @brief Subsample Reference Trajectory.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] initTime: Init Time.
   * @param [in] finalTime: Final Time.
   */
  void SubsampleReferenceTrajectory(const bolza::TargetTrajectories& targetTrajectories, scalar_t initTime, scalar_t finalTime);

  // Apply IK to cartesian swing motion to update joint references
  /**
   * @brief Adapt Joint References With Inverse Kinematics.
   * @param [in] finalTime: Final Time.
   */
  void AdaptJointReferencesWithInverseKinematics(scalar_t finalTime);

  /**
   * @brief Extract External Swing Phase.
   * @param [in] leg: Leg.
   * @param [in] liftOffTime: Lift Off Time.
   * @param [in] touchDownTime: Touch Down Time.
   * @return Owning pointer.
   */
  std::unique_ptr<ExternalSwingPhase> ExtractExternalSwingPhase(int leg, scalar_t liftOffTime, scalar_t touchDownTime) const;

  /**
   * @brief Get Default Swing Profile.
   * @return Result value.
   */
  SwingPhase::SwingProfile GetDefaultSwingProfile() const;

  /**
   * @brief Get Contact End Time.
   * @param [in] contactPhase: Contact Phase.
   * @param [in] finalTime: Final Time.
   * @return Result value.
   */
  scalar_t GetContactEndTime(const ContactTiming& contactPhase, scalar_t finalTime) const;

  /**
   * @brief Get Foot Phase If In Contact.
   * @param [in] leg: Leg.
   * @param [in] time: Time.
   * @return Pointer; may be null.
   */
  const FootPhase* GetFootPhaseIfInContact(size_t leg, scalar_t time) const;

  /**
   * @brief Filter Foothold.
   * @param [in] newFoothold: New Foothold.
   * @param [in] previousFoothold: Previous Foothold.
   * @return Result value.
   */
  vector3_t FilterFoothold(const vector3_t& newFoothold, const vector3_t& previousFoothold) const;

  SwingTrajectoryPlannerSettings settings_;
  std::unique_ptr<KinematicsModelBase<scalar_t>> kinematicsModel_;
  std::unique_ptr<InverseKinematicsModelBase> inverseKinematicsModelPtr_;

  feet_array_t<std::pair<scalar_t, TerrainPlane>> lastContacts_;
  feet_array_t<std::vector<std::unique_ptr<FootPhase>>> feetNormalTrajectories_;
  feet_array_t<std::vector<scalar_t>> feetNormalTrajectoriesEvents_;

  feet_array_t<std::vector<ConvexTerrain>> nominalFootholdsPerLeg_;
  feet_array_t<std::vector<vector3_t>> heuristicFootholdsPerLeg_;
  std::unique_ptr<TerrainModel> terrainModel_;

  bolza::TargetTrajectories targetTrajectories_;
};

}  // namespace switched_model
