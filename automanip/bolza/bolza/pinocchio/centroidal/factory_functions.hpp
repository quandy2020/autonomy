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

/**
 * @file factory_functions.hpp
 * @brief Create a CentroidalModel PinocchioInterface from a URDF.
 */


#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>
#include "bolza/pinocchio/interface/pinocchio_interface.hpp"

#include "bolza/pinocchio/centroidal/centroidal_model_info.hpp"

namespace bolza {
namespace centroidal_model {

/**
 * @brief Create a CentroidalModel PinocchioInterface from a URDF.
 * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
 * @return Result value.
 */
PinocchioInterface CreatePinocchioInterface(const std::string& urdfFilePath);

/**
 * @brief Create a CentroidalModel PinocchioInterface from a URDF.
 * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
 * @param [in] jointNames: Any joint that is not listed in jointNames (a.k.a the extraneous joints) will be removed from the urdf.
 * @return Result value.
 */
PinocchioInterface CreatePinocchioInterface(const std::string& urdfFilePath, const std::vector<std::string>& jointNames);

/**
 * Create a scalar-typed CentroidalModelInfo.
 * @param [in] interface: Pinocchio interface
 * @param [in] type: Type of template model (SRBD or FRBD)
 * @param [in] nominalJointAngles: nominal joint angles used in the SRBD model.
 * @param [in] threeDofContactNames: Names of end-effectors with 3 DoF contacts (force)
 * @param [in] sixDofContactNames: Names of end-effectors with 6 DoF contacts (force + torque)
 * @return CentroidalModelInfo
 */
CentroidalModelInfo CreateCentroidalModelInfo(const PinocchioInterface& interface, const CentroidalModelType& type,
                                              const vector_t& nominalJointAngles, const std::vector<std::string>& threeDofContactNames,
                                              const std::vector<std::string>& sixDofContactNames);

/**
 * @brief Load CentroidalModelType for a config file.
 * @param [in] configFilePath: Config File Path.
 * @param [in] fieldName: Field Name.
 * @return Result value.
 */
CentroidalModelType LoadCentroidalType(const std::string& configFilePath, const std::string& fieldName = "centroidalModelType");

/**
 * @brief Load default joint state for a config file.
 * @param [in] numJointState: Num Joint State.
 * @param [in] configFilePath: Config File Path.
 * @param [in] fieldName: Field Name.
 * @return Result value.
 */
vector_t LoadDefaultJointState(size_t numJointState, const std::string& configFilePath, const std::string& fieldName = "defaultJointState");

}  // namespace centroidal_model
}  // namespace bolza
