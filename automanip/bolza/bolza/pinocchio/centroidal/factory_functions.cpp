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

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "bolza/pinocchio/centroidal/factory_functions.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

#include <bolza/core/misc/load_data.hpp>
#include "bolza/pinocchio/interface/urdf.hpp"

namespace bolza {
namespace centroidal_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface CreatePinocchioInterface(const std::string& urdfFilePath) {
  // add 6 DoF for the floating base
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());

  return GetPinocchioInterfaceFromUrdfFile(urdfFilePath, jointComposite);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface CreatePinocchioInterface(const std::string& urdfFilePath, const std::vector<std::string>& jointNames) {
  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFilePath);
  if (urdfTree == nullptr) {
    throw std::invalid_argument("The file " + urdfFilePath + " does not contain a valid URDF model!");
  }

  // remove extraneous joints from urdf
  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  for (joint_pair_t& jointPair : newModel->joints_) {
    if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) == jointNames.end()) {
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }

  // add 6 DoF for the floating base
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());

  return GetPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelInfo CreateCentroidalModelInfo(const PinocchioInterface& interface, const CentroidalModelType& type,
                                              const vector_t& nominalJointAngles, const std::vector<std::string>& threeDofContactNames,
                                              const std::vector<std::string>& sixDofContactNames) {
  const auto& model = interface.getModel();
  auto data = interface.getData();

  if (model.nq != nominalJointAngles.size() + 6) {
    const int expaectedNumJoints = model.nq - 6;
    throw std::runtime_error("[CentroidalModelInfo] nominalJointAngles.size() should be " + std::to_string(expaectedNumJoints));
  }

  CentroidalModelInfoTpl<scalar_t> info;
  info.centroidalModelType = type;
  info.numThreeDofContacts = threeDofContactNames.size();
  info.numSixDofContacts = sixDofContactNames.size();
  info.generalizedCoordinatesNum = model.nq;
  info.actuatedDofNum = info.generalizedCoordinatesNum - 6;
  info.stateDim = info.generalizedCoordinatesNum + 6;
  info.inputDim = info.actuatedDofNum + 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  info.robotMass = pinocchio::computeTotalMass(model);

  for (const auto& name : threeDofContactNames) {
    info.endEffectorFrameIndices.push_back(model.getBodyId(name));
  }

  for (const auto& name : sixDofContactNames) {
    info.endEffectorFrameIndices.push_back(model.getBodyId(name));
  }

  // make sure the nominal base frame is aligned with the world frame
  info.qPinocchioNominal.resize(model.nq);
  info.qPinocchioNominal << vector_t::Zero(6), nominalJointAngles;
  info.centroidalInertiaNominal.setZero();
  info.comToBasePositionNominal.setZero();
  if (info.centroidalModelType == CentroidalModelType::SingleRigidBodyDynamics) {
    const vector_t vPinocchioNominal = vector_t::Zero(info.generalizedCoordinatesNum);
    pinocchio::ccrba(model, data, info.qPinocchioNominal, vPinocchioNominal);
    info.centroidalInertiaNominal = data.Ig.inertia().matrix();
    info.comToBasePositionNominal = info.qPinocchioNominal.template head<3>() - data.com[0];
  }

  return info;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelType LoadCentroidalType(const std::string& configFilePath, const std::string& fieldName) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(configFilePath, pt);
  const size_t type = pt.template get<size_t>(fieldName);
  return static_cast<CentroidalModelType>(type);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LoadDefaultJointState(size_t numJointState, const std::string& configFilePath, const std::string& fieldName) {
  vector_t defaultJoints(numJointState);
  bolza::loadData::LoadEigenMatrix(configFilePath, fieldName, defaultJoints);
  return defaultJoints;
}

}  // namespace centroidal_model
}  // namespace bolza
