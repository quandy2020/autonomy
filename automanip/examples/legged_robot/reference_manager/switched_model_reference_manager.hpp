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

#include <map>
#include <mutex>
#include <string>

#include <bolza/core/thread_support/synchronized.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>

#include "examples/legged_robot/foot_planner/swing_trajectory_planner.hpp"
#include "examples/legged_robot/gait/gait_schedule.hpp"
#include "examples/legged_robot/gait/motion_phase_definition.hpp"
#include "examples/legged_robot/terrain/terrain.hpp"

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
  void SetTargetTrajectories(const TargetTrajectories& targetTrajectories) override;
  void SetTargetTrajectories(TargetTrajectories&& targetTrajectories) override;

  contact_flag_t GetContactFlags(scalar_t time) const;

  /** Names match `config/command/gait.info`. `stance` stands. Any other loaded name is the walk gait. */
  void LoadGaits(const std::string& gait_file);
  void SetGait(const std::string& name);
  std::string GaitList() const;

  /** `terrain.info`, or a runtime command: `flat`, `stairs`, or a height-file path. */
  void LoadTerrain(const std::string& terrain_file);
  void SetTerrainCommand(const std::string& command);
  Terrain TerrainCopy() const;

  /** Foot position relative to the base, order LF, RF, LH, RH. */
  void SetFootOffsets(const feet_array_t<vector3_t>& offsets);

  /** Unit normal of the surface under this foot. Order matches the contact list. */
  vector3_t SurfaceNormal(size_t foot) const;

  const std::shared_ptr<GaitSchedule>& GetGaitSchedule() { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& GetSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

 private:
  void ModifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;
  void RememberCommand(const TargetTrajectories& targetTrajectories);

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
  // Stance locks every foot. A base target then has no feasible step, and the
  // rollout diverges. The selected locomotion gait runs only while the base is
  // away from the goal, or while an explicit gait command is held.
  bool walking_ = false;
  bool hold_gait_ = false;
  bool force_stance_ = false;
  bool gait_dirty_ = false;
  std::string locomotion_ = "trot";
  std::string terrain_file_;
  std::map<std::string, ModeSequenceTemplate> gaits_;
  TargetTrajectories command_;
  Terrain terrain_ = Terrain::flat();
  feet_array_t<vector3_t> foot_offsets_{};
  feet_array_t<vector3_t> surface_normals_{};
  mutable std::mutex mutex_;
};

}  // namespace legged_robot
}  // namespace bolza
