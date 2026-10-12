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

// OCS2
#include <bolza/core/types.hpp>
#include <bolza/core/initialization/initializer.hpp>
#include <bolza/ddp/ddp_settings.hpp>
#include <bolza/mpc/mpc_settings.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>
#include <bolza/tools/common/robot_interface.hpp>

#include "examples/mobile_manipulator/factory_functions.hpp"
#include <bolza/pinocchio/interface/pinocchio_interface.hpp>

namespace bolza {
namespace mobile_manipulator {

/**
 * Mobile Manipulator Robot Interface class
 */
class MobileManipulatorInterface final : public RobotInterface {
 public:
  /**
   * Constructor
   *
   * @note Creates directory for generated library into if it does not exist.
   * @throw Invalid argument error if input task file or urdf file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
   * @param [in] urdfFile: The absolute path to the URDF file for the robot.
   */
  MobileManipulatorInterface(const std::string& taskFile, const std::string& libraryFolder, const std::string& urdfFile);

  const vector_t& GetInitialState() { return initialState_; }

  ddp::Settings& DdpSettings() { return ddpSettings_; }

  mpc::Settings& MpcSettings() { return mpcSettings_; }

  const OptimalControlProblem& GetOptimalControlProblem() const override { return problem_; }

  std::shared_ptr<ReferenceManagerInterface> GetReferenceManagerPtr() const override { return referenceManagerPtr_; }

  const Initializer& GetInitializer() const override { return *initializerPtr_; }

  const RolloutBase& GetRollout() const { return *rolloutPtr_; }

  const PinocchioInterface& GetPinocchioInterface() const { return *pinocchioInterfacePtr_; }

  const ManipulatorModelInfo& GetManipulatorModelInfo() const { return manipulatorModelInfo_; }

 private:
  std::unique_ptr<StateInputCost> GetQuadraticInputCost(const std::string& taskFile);
  std::unique_ptr<StateCost> GetEndEffectorConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                      const std::string& prefix, bool useCaching, const std::string& libraryFolder,
                                                      bool recompileLibraries);
  std::unique_ptr<StateCost> GetSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                        const std::string& urdfFile, const std::string& prefix, bool useCaching,
                                                        const std::string& libraryFolder, bool recompileLibraries);
  std::unique_ptr<StateInputCost> GetJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile);

  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;

  OptimalControlProblem problem_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> initializerPtr_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  ManipulatorModelInfo manipulatorModelInfo_;

  vector_t initialState_;
};

}  // namespace mobile_manipulator
}  // namespace bolza
