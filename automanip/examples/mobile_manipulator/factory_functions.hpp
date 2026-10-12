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

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>
#include <bolza/pinocchio/interface/pinocchio_interface.hpp>

#include "examples/mobile_manipulator/manipulator_model_info.hpp"

namespace bolza {
namespace mobile_manipulator {

/** Create a MobileManipulatorModel PinocchioInterface from a URDF
 * @param [in] robotUrdfPath: The robot URDF path.
 * @param [in] type: Type of robot model (fixed-arm or wheel-based)
 * @return PinocchioInterface
 */
PinocchioInterface CreatePinocchioInterface(const std::string& robotUrdfPath, const ManipulatorModelType& type);

/** Create a MobileManipulatorModel PinocchioInterface from a URDF
 * @param [in] robotUrdfPath: The robot URDF path.
 * @param [in] type: Type of robot model (fixed-arm or wheel-based)
 * @param [in] jointNames: The joint names from URDF to make fixed/unactuated.
 * @return PinocchioInterface
 */
PinocchioInterface CreatePinocchioInterface(const std::string& robotUrdfPath, const ManipulatorModelType& type,
                                            const std::vector<std::string>& jointNames);

/**
 * Create a scalar-typed ManipulatorModelInfo.
 * @param [in] interface: Pinocchio interface
 * @param [in] type: Type of template model (default-arm or wheel-based or floating-arm)
 * @param [in] baseFrame: Name of the root frame.
 * @param [in] eeFrame: Name of the end-effector frame.
 * @return ManipulatorModelInfo
 */
ManipulatorModelInfo CreateManipulatorModelInfo(const PinocchioInterface& interface, const ManipulatorModelType& type,
                                                const std::string& baseFrame, const std::string& eeFrame);

/** Load ManipulatorModelType for a config file */
ManipulatorModelType LoadManipulatorType(const std::string& configFilePath, const std::string& fieldName = "manipulatorModelType");

}  // namespace mobile_manipulator
}  // namespace bolza
