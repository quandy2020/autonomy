/**
 * @file urdf.hpp
 * @brief Factory function from URDF file.
 */

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

#include <string>

#include "bolza/pinocchio/interface/pinocchio_interface.hpp"
#include <urdf_model/model.h>

namespace bolza {

/**
 * @brief Factory function from URDF file.
 * @param [in] urdfFile: Path to the URDF file.
 * @return Result value.
 */
PinocchioInterface GetPinocchioInterfaceFromUrdfFile(const std::string& urdfFile);

/**
 * @brief Factory function from URDF file with root joint.
 * @param [in] urdfFile: Path to the URDF file.
 * @param [in] rootJoint: Root joint to which the robot is attached (eg. a 6 DOF base).
 * @return Result value.
 */
PinocchioInterface GetPinocchioInterfaceFromUrdfFile(const std::string& urdfFile, const PinocchioInterface::JointModel& rootJoint);

/**
 * @brief Factory function from URDF string.
 * @param [in] xmlString: A URDF loaded as an std::string.
 * @return Result value.
 */
PinocchioInterface GetPinocchioInterfaceFromUrdfString(const std::string& xmlString);

/**
 * @brief Factory function from URDF string with root joint.
 * @param [in] xmlString: A URDF loaded as an std::string.
 * @param [in] rootJoint: Root joint to which the robot is attached (eg. a 6 DOF base).
 * @return Result value.
 */
PinocchioInterface GetPinocchioInterfaceFromUrdfString(const std::string& xmlString, const PinocchioInterface::JointModel& rootJoint);

/**
 * @brief Factory function from URDF model tree.
 * @param [in] urdfTree: Pointer to a URDF model tree.
 * @return Result value.
 */
PinocchioInterface GetPinocchioInterfaceFromUrdfModel(const std::shared_ptr<::urdf::ModelInterface>& urdfTree);

/**
 * @brief Factory function from URDF model tree with root joint.
 * @param [in] urdfTree: Pointer to a URDF model tree.
 * @param [in] rootJoint: Root joint to which the robot is attached (eg. a 6 DOF base).
 * @return Result value.
 */
PinocchioInterface GetPinocchioInterfaceFromUrdfModel(const std::shared_ptr<::urdf::ModelInterface>& urdfTree,
                                                      const PinocchioInterface::JointModel& rootJoint);
}  // namespace bolza
