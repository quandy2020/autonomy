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

// ocs2
#include <bolza/pinocchio/centroidal/factory_functions.hpp>
#include <bolza/core/types.hpp>
#include <bolza/core/penalties/penalties.hpp>
#include <bolza/ddp/ddp_settings.hpp>
#include <bolza/ipm/ipm_settings.hpp>
#include <bolza/mpc/mpc_settings.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>
#include <bolza/pinocchio/interface/pinocchio_interface.hpp>
#include <bolza/tools/common/robot_interface.hpp>
#include <bolza/tools/end_effector/end_effector_kinematics.hpp>
#include <bolza/sqp/sqp_settings.hpp>

#include "examples/legged_robot/common/model_settings.hpp"
#include "examples/legged_robot/initialization/legged_robot_initializer.hpp"
#include "examples/legged_robot/reference_manager/switched_model_reference_manager.hpp"

/**
 * LeggedRobotInterface class
 * General interface for mpc implementation on the legged robot model
 */
namespace bolza {
namespace legged_robot {

class LeggedRobotInterface final : public RobotInterface {
 public:
  /**
   * Constructor
   *
   * @throw Invalid argument error if input task file or urdf file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] urdfFile: The absolute path to the URDF file for the robot.
   * @param [in] referenceFile: The absolute path to the reference configuration file.
   * @param [in] useHardFrictionConeConstraint: Which to use hard or soft friction cone constraints.
   */
  LeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                       bool useHardFrictionConeConstraint = false);

  ~LeggedRobotInterface() override = default;

  const OptimalControlProblem& GetOptimalControlProblem() const override { return *problemPtr_; }

  const ModelSettings& ModelSettings() const { return modelSettings_; }
  const ddp::Settings& DdpSettings() const { return ddpSettings_; }
  const mpc::Settings& MpcSettings() const { return mpcSettings_; }
  const rollout::Settings& RolloutSettings() const { return rolloutSettings_; }
  const sqp::Settings& SqpSettings() { return sqpSettings_; }
  const ipm::Settings& IpmSettings() { return ipmSettings_; }

  const vector_t& GetInitialState() const { return initialState_; }
  const RolloutBase& GetRollout() const { return *rolloutPtr_; }
  PinocchioInterface& GetPinocchioInterface() { return *pinocchioInterfacePtr_; }
  const CentroidalModelInfo& GetCentroidalModelInfo() const { return centroidalModelInfo_; }
  std::shared_ptr<SwitchedModelReferenceManager> GetSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const LeggedRobotInitializer& GetInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> GetReferenceManagerPtr() const override { return referenceManagerPtr_; }

 private:
  void SetupOptimalConrolProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);

  std::shared_ptr<GaitSchedule> LoadGaitSchedule(const std::string& file, bool verbose) const;

  std::unique_ptr<StateInputCost> GetBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info, bool verbose);
  matrix_t InitializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);

  std::pair<scalar_t, RelaxedBarrierPenalty::Config> LoadFrictionConeSettings(const std::string& taskFile, bool verbose) const;
  std::unique_ptr<StateInputConstraint> GetFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient);
  std::unique_ptr<StateInputCost> GetFrictionConeSoftConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  std::unique_ptr<StateInputConstraint> GetZeroForceConstraint(size_t contactPointIndex);
  std::unique_ptr<StateInputConstraint> GetZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  size_t contactPointIndex, bool useAnalyticalGradients);
  std::unique_ptr<StateInputConstraint> GetNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                    size_t contactPointIndex, bool useAnalyticalGradients);

  ModelSettings modelSettings_;
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;
  ipm::Settings ipmSettings_;
  const bool useHardFrictionConeConstraint_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<LeggedRobotInitializer> initializerPtr_;

  vector_t initialState_;
};

}  // namespace legged_robot
}  // namespace bolza
