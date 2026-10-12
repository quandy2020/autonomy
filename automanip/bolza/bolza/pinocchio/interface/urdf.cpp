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

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <urdf_parser/urdf_parser.h>

#include "bolza/pinocchio/interface/urdf.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface GetPinocchioInterfaceFromUrdfFile(const std::string& urdfFile) {
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFile);
  if (urdfTree != nullptr) {
    return GetPinocchioInterfaceFromUrdfModel(urdfTree);
  } else {
    throw std::invalid_argument("The file " + urdfFile + " does not contain a valid URDF model.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface GetPinocchioInterfaceFromUrdfFile(const std::string& urdfFile, const PinocchioInterface::JointModel& rootJoint) {
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFile);
  if (urdfTree != nullptr) {
    return GetPinocchioInterfaceFromUrdfModel(urdfTree, rootJoint);
  } else {
    throw std::invalid_argument("The file " + urdfFile + " does not contain a valid URDF model.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface GetPinocchioInterfaceFromUrdfString(const std::string& xmlString) {
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(xmlString);
  if (urdfTree != nullptr) {
    return GetPinocchioInterfaceFromUrdfModel(urdfTree);
  } else {
    throw std::invalid_argument("The XML stream does not contain a valid URDF model.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface GetPinocchioInterfaceFromUrdfString(const std::string& xmlString, const PinocchioInterface::JointModel& rootJoint) {
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(xmlString);
  if (urdfTree != nullptr) {
    return GetPinocchioInterfaceFromUrdfModel(urdfTree, rootJoint);
  } else {
    throw std::invalid_argument("The XML stream does not contain a valid URDF model.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface GetPinocchioInterfaceFromUrdfModel(const std::shared_ptr<::urdf::ModelInterface>& urdfTree) {
  pinocchio::ModelTpl<scalar_t> model;
  pinocchio::urdf::buildModel(urdfTree, model);
  return PinocchioInterface(model, urdfTree);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface GetPinocchioInterfaceFromUrdfModel(const std::shared_ptr<::urdf::ModelInterface>& urdfTree,
                                                      const PinocchioInterface::JointModel& rootJoint) {
  pinocchio::ModelTpl<scalar_t> model;
  pinocchio::urdf::buildModel(urdfTree, rootJoint, model);
  return PinocchioInterface(model, urdfTree);
}

}  // namespace bolza
