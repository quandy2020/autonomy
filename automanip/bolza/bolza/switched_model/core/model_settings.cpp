//
// Created by rgrandia on 24.10.19.
//

#include "bolza/switched_model/core/model_settings.hpp"

#include <iostream>
#include <unordered_map>


#include <bolza/core/misc/load_data.hpp>

namespace switched_model {

std::string ToAlgorithmName(Algorithm type) {
  static const std::unordered_map<Algorithm, std::string> map{{Algorithm::DDP, "DDP"}, {Algorithm::SQP, "SQP"}};
  return map.at(type);
}

Algorithm FromAlgorithmName(std::string name) {
  static const std::unordered_map<std::string, Algorithm> map{{"DDP", Algorithm::DDP}, {"SQP", Algorithm::SQP}};
  std::transform(name.begin(), name.end(), name.begin(), ::toupper);
  return map.at(name);
}

ModelSettings LoadModelSettings(const std::string& filename, bool verbose) {
  ModelSettings modelSettings;

  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  const std::string prefix{"model_settings."};

  if (verbose) {
    std::cerr << "\n #### Robot Model Settings:" << std::endl;
    std::cerr << " #### ==================================================" << std::endl;
  }

  std::string algorithmName = ToAlgorithmName(modelSettings.algorithm_);
  bolza::loadData::LoadPtreeValue(pt, algorithmName, prefix + "algorithm", verbose);
  modelSettings.algorithm_ = FromAlgorithmName(algorithmName);

  bolza::loadData::LoadPtreeValue(pt, modelSettings.phaseTransitionStanceTime_, prefix + "phaseTransitionStanceTime", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.recompileLibraries_, prefix + "recompileLibraries", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.robotName_, prefix + "robotName", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.autodiffLibraryFolder_, prefix + "autodiffLibraryFolder", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.analyticalInverseKinematics_, prefix + "analyticalInverseKinematics", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.frictionCoefficient_, prefix + "frictionCoefficient", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.coneRegularization_, prefix + "coneRegularization", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.gripperForce_, prefix + "gripperForce", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.muFrictionCone_, prefix + "muFrictionCone", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.deltaFrictionCone_, prefix + "deltaFrictionCone", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.muFootPlacement_, prefix + "muFootPlacement", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.deltaFootPlacement_, prefix + "deltaFootPlacement", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.muSdf_, prefix + "muSdf", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.deltaSdf_, prefix + "deltaSdf", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.muJointsPosition_, prefix + "muJointsPosition", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.deltaJointsPosition_, prefix + "deltaJointsPosition", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.muJointsVelocity_, prefix + "muJointsVelocity", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.deltaJointsVelocity_, prefix + "deltaJointsVelocity", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.muJointsTorque_, prefix + "muJointsTorque", verbose);
  bolza::loadData::LoadPtreeValue(pt, modelSettings.deltaJointsTorque_, prefix + "deltaJointsTorque", verbose);
  bolza::loadData::LoadEigenMatrix(filename, prefix + "joint_lower_limits", modelSettings.lowerJointLimits_);
  bolza::loadData::LoadEigenMatrix(filename, prefix + "joint_upper_limits", modelSettings.upperJointLimits_);
  bolza::loadData::LoadEigenMatrix(filename, prefix + "joint_velocity_limits", modelSettings.jointVelocityLimits);
  bolza::loadData::LoadEigenMatrix(filename, prefix + "joint_torque_limits", modelSettings.jointTorqueLimits);

  if (verbose) {
    std::cerr << " joint lower limits: " << modelSettings.lowerJointLimits_.transpose() << "\n";
    std::cerr << " joint upper limits: " << modelSettings.upperJointLimits_.transpose() << "\n";
    std::cerr << " joint velocity limits: " << modelSettings.jointVelocityLimits.transpose() << "\n";
    std::cerr << " joint torque limits: " << modelSettings.jointTorqueLimits.transpose() << "\n";
  }

  if (verbose) {
    std::cerr << " #### ================================================ ####" << std::endl;
  }

  return modelSettings;
}

}  // namespace switched_model
