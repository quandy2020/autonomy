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

#include "examples/mobile_manipulator/factory_functions.hpp"

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>


#include <bolza/core/misc/load_data.hpp>
#include <bolza/pinocchio/interface/urdf.hpp>

namespace bolza {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface CreatePinocchioInterface(const std::string& robotUrdfPath, const ManipulatorModelType& type) {
  switch (type) {
    case ManipulatorModelType::DefaultManipulator: {
      // return pinocchio interface
      return GetPinocchioInterfaceFromUrdfFile(robotUrdfPath);
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      // add 6 DoF for the floating base
      pinocchio::JointModelComposite jointComposite(2);
      jointComposite.addJoint(pinocchio::JointModelTranslation());
      jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
      // return pinocchio interface
      return GetPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      // add 6 DOF for the fully-actuated free-floating base
      pinocchio::JointModelComposite jointComposite(2);
      jointComposite.addJoint(pinocchio::JointModelTranslation());
      jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
      // return pinocchio interface
      return GetPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      // add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());
      // return pinocchio interface
      return GetPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface CreatePinocchioInterface(const std::string& robotUrdfPath, const ManipulatorModelType& type,
                                            const std::vector<std::string>& jointNames) {
  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

  // parse the URDF
  const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
  // remove extraneous joints from urdf
  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  for (joint_pair_t& jointPair : newModel->joints_) {
    if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) != jointNames.end()) {
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }
  // resolve for the robot type
  switch (type) {
    case ManipulatorModelType::DefaultManipulator: {
      // return pinocchio interface
      return GetPinocchioInterfaceFromUrdfModel(newModel);
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      // add 6 DoF for the floating base
      pinocchio::JointModelComposite jointComposite(2);
      jointComposite.addJoint(pinocchio::JointModelTranslation());
      jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
      // return pinocchio interface
      return GetPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      // add 6 DOF for the free-floating base
      pinocchio::JointModelComposite jointComposite(2);
      jointComposite.addJoint(pinocchio::JointModelTranslation());
      jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
      // return pinocchio interface
      return GetPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      // add XY-yaw joint for the wheel-base
      pinocchio::JointModelComposite jointComposite(3);
      jointComposite.addJoint(pinocchio::JointModelPX());
      jointComposite.addJoint(pinocchio::JointModelPY());
      jointComposite.addJoint(pinocchio::JointModelRZ());
      // return pinocchio interface
      return GetPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ManipulatorModelInfo CreateManipulatorModelInfo(const PinocchioInterface& interface, const ManipulatorModelType& type,
                                                const std::string& baseFrame, const std::string& eeFrame) {
  const auto& model = interface.GetModel();

  ManipulatorModelInfo info;
  info.manipulatorModelType = type;
  info.stateDim = model.nq;
  // resolve for actuated dof based on type of robot
  switch (type) {
    case ManipulatorModelType::DefaultManipulator: {
      // for default arm, the state dimension and input dimensions are same.
      info.inputDim = info.stateDim;
      info.armDim = info.inputDim;
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      // remove the static 6-DOF base joints that are unactuated.
      info.inputDim = info.stateDim - 6;
      info.armDim = info.inputDim;
      break;
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      // all states are actuatable
      info.inputDim = info.stateDim;
      info.armDim = info.inputDim - 6;
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      // for wheel-based, the input dimension is (v, omega, dq_j) while state dimension is (x, y, psi, q_j).
      info.inputDim = info.stateDim - 1;
      info.armDim = info.inputDim - 2;
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
      break;
  }
  // store frame names for using later.
  info.eeFrame = eeFrame;
  info.baseFrame = baseFrame;
  // get name of arm joints.
  const auto& jointNames = model.names;
  info.dofNames = std::vector<std::string>(jointNames.end() - info.armDim, jointNames.end());

  return info;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ManipulatorModelType LoadManipulatorType(const std::string& configFilePath, const std::string& fieldName) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(configFilePath, pt);
  const size_t type = pt.template get<size_t>(fieldName);
  return static_cast<ManipulatorModelType>(type);
}

}  // namespace mobile_manipulator
}  // namespace bolza