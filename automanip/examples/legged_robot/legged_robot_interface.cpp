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

#include <cmath>
#include <iostream>
#include <filesystem>
#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "examples/legged_robot/legged_robot_interface.hpp"

#include <bolza/pinocchio/centroidal/access_helper_functions.hpp>
#include <bolza/pinocchio/centroidal/centroidal_model_pinocchio_mapping.hpp>
#include <bolza/pinocchio/centroidal/model_helper_functions.hpp>
#include <bolza/core/misc/display.hpp>
#include <bolza/core/soft_constraint/state_input_soft_constraint.hpp>
#include <bolza/oc/synchronized_module/solver_synchronized_module.hpp>
#include <bolza/pinocchio/interface/pinocchio_end_effector_kinematics_cpp_ad.hpp>

#include "examples/legged_robot/legged_robot_pre_computation.hpp"
#include "examples/legged_robot/constraint/friction_cone_constraint.hpp"
#include "examples/legged_robot/constraint/normal_velocity_constraint_cpp_ad.hpp"
#include "examples/legged_robot/constraint/zero_force_constraint.hpp"
#include "examples/legged_robot/constraint/zero_velocity_constraint_cpp_ad.hpp"
#include "examples/legged_robot/cost/legged_robot_quadratic_tracking_cost.hpp"
#include "examples/legged_robot/dynamics/legged_robot_dynamics_ad.hpp"

// Boost

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInterface::LeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                           bool useHardFrictionConeConstraint)
    : useHardFrictionConeConstraint_(useHardFrictionConeConstraint) {
  // check that task file exists
  std::filesystem::path taskFilePath(taskFile);
  if (std::filesystem::exists(taskFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  std::filesystem::path urdfFilePath(urdfFile);
  if (std::filesystem::exists(urdfFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] URDF file not found: " + urdfFilePath.string());
  }
  // check that targetCommand file exists
  std::filesystem::path referenceFilePath(referenceFile);
  if (std::filesystem::exists(referenceFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  bool verbose;
  loadData::LoadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // load setting from loading file
  modelSettings_ = LoadModelSettings(taskFile, "model_settings", verbose);
  mpcSettings_ = mpc::LoadSettings(taskFile, "mpc", verbose);
  ddpSettings_ = ddp::LoadSettings(taskFile, "ddp", verbose);
  sqpSettings_ = sqp::LoadSettings(taskFile, "sqp", verbose);
  ipmSettings_ = ipm::LoadSettings(taskFile, "ipm", verbose);
  rolloutSettings_ = rollout::LoadSettings(taskFile, "rollout", verbose);

  // OptimalConrolProblem
  SetupOptimalConrolProblem(taskFile, urdfFile, referenceFile, verbose);

  // initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::LoadEigenMatrix(taskFile, "initialState", initialState_);

  const std::filesystem::path command_dir = referenceFilePath.parent_path();
  referenceManagerPtr_->LoadGaits((command_dir / "gait.info").string());
  referenceManagerPtr_->LoadTerrain((command_dir.parent_path() / "terrain" / "terrain.info").string());

  auto& model = pinocchioInterfacePtr_->GetModel();
  auto& data = pinocchioInterfacePtr_->GetData();
  const auto q = centroidal_model::getGeneralizedCoordinates(initialState_, centroidalModelInfo_);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  const double yaw = initialState_.size() > 9 ? initialState_(9) : 0.0;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  feet_array_t<vector3_t> offsets;
  offsets[0] = vector3_t(0.30, 0.20, 0.0);
  offsets[1] = vector3_t(0.30, -0.20, 0.0);
  offsets[2] = vector3_t(-0.30, 0.20, 0.0);
  offsets[3] = vector3_t(-0.30, -0.20, 0.0);
  for (std::size_t i = 0; i < offsets.size() && i < modelSettings_.contactNames3DoF.size(); ++i) {
    if (!model.existFrame(modelSettings_.contactNames3DoF[i])) {
      continue;
    }
    const auto frame = model.getFrameId(modelSettings_.contactNames3DoF[i]);
    const auto position = data.oMf[frame].translation();
    const double dx = position.x() - initialState_(6);
    const double dy = position.y() - initialState_(7);
    offsets[i].x() = c * dx + s * dy;
    offsets[i].y() = -s * dx + c * dy;
    offsets[i].z() = 0.0;
  }
  referenceManagerPtr_->SetFootOffsets(offsets);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInterface::SetupOptimalConrolProblem(const std::string& taskFile, const std::string& urdfFile,
                                                     const std::string& referenceFile, bool verbose) {
  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(centroidal_model::CreatePinocchioInterface(urdfFile, modelSettings_.jointNames)));

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::CreateCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::LoadCentroidalType(taskFile),
      centroidal_model::LoadDefaultJointState(pinocchioInterfacePtr_->GetModel().nq - 6, referenceFile), modelSettings_.contactNames3DoF,
      modelSettings_.contactNames6DoF);

  // Swing trajectory planner
  auto swingTrajectoryPlanner =
      std::make_unique<SwingTrajectoryPlanner>(LoadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);

  // Mode schedule manager
  referenceManagerPtr_ =
      std::make_shared<SwitchedModelReferenceManager>(LoadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner));

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  bool useAnalyticalGradientsDynamics = false;
  loadData::LoadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsDynamics", useAnalyticalGradientsDynamics);
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  if (useAnalyticalGradientsDynamics) {
    throw std::runtime_error("[LeggedRobotInterface::SetupOptimalConrolProblem] The analytical dynamics class is not yet implemented!");
  } else {
    const std::string modelName = "dynamics";
    dynamicsPtr.reset(new LeggedRobotDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelName, modelSettings_));
  }

  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  problemPtr_->costPtr->Add("baseTrackingCost", GetBaseTrackingCost(taskFile, centroidalModelInfo_, false));

  // Constraint terms
  // friction cone settings
  scalar_t frictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  std::tie(frictionCoefficient, barrierPenaltyConfig) = LoadFrictionConeSettings(taskFile, verbose);

  bool useAnalyticalGradientsConstraints = false;
  loadData::LoadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints", useAnalyticalGradientsConstraints);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings_.contactNames3DoF[i];

    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;
    if (useAnalyticalGradientsConstraints) {
      throw std::runtime_error(
          "[LeggedRobotInterface::SetupOptimalConrolProblem] The analytical end-effector linear constraint is not implemented!");
    } else {
      const auto infoCppAd = centroidalModelInfo_.ToCppAd();
      const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
      auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
        const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
        UpdateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
      };
      eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
                                                                    centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                    velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
                                                                    modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
    }

    if (useHardFrictionConeConstraint_) {
      problemPtr_->inequalityConstraintPtr->Add(footName + "_frictionCone", GetFrictionConeConstraint(i, frictionCoefficient));
    } else {
      problemPtr_->softConstraintPtr->Add(footName + "_frictionCone",
                                          GetFrictionConeSoftConstraint(i, frictionCoefficient, barrierPenaltyConfig));
    }
    problemPtr_->equalityConstraintPtr->Add(footName + "_zeroForce", GetZeroForceConstraint(i));
    problemPtr_->equalityConstraintPtr->Add(footName + "_zeroVelocity",
                                            GetZeroVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
    problemPtr_->equalityConstraintPtr->Add(footName + "_normalVelocity",
                                            GetNormalVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
  }

  // Pre-computation
  problemPtr_->preComputationPtr.reset(new LeggedRobotPreComputation(*pinocchioInterfacePtr_, centroidalModelInfo_,
                                                                     *referenceManagerPtr_->GetSwingTrajectoryPlanner(), modelSettings_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_.reset(new LeggedRobotInitializer(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedRobotInterface::LoadGaitSchedule(const std::string& file, bool verbose) const {
  const auto initModeSchedule = LoadModeSchedule(file, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = LoadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

  const auto defaultGait = [&] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  // display
  if (verbose) {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
    std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings_.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LeggedRobotInterface::InitializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info) {
  const size_t totalContactDim = 3 * info.numThreeDofContacts;

  vector_t initialState(centroidalModelInfo_.stateDim);
  loadData::LoadEigenMatrix(taskFile, "initialState", initialState);

  const auto& model = pinocchioInterfacePtr_->GetModel();
  auto& data = pinocchioInterfacePtr_->GetData();
  const auto q = centroidal_model::getGeneralizedCoordinates(initialState, centroidalModelInfo_);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  matrix_t baseToFeetJacobians(totalContactDim, info.actuatedDofNum);
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    matrix_t jacobianWorldToContactPointInWorldFrame = matrix_t::Zero(6, info.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]), pinocchio::LOCAL_WORLD_ALIGNED,
                                jacobianWorldToContactPointInWorldFrame);

    baseToFeetJacobians.block(3 * i, 0, 3, info.actuatedDofNum) =
        jacobianWorldToContactPointInWorldFrame.block(0, 6, 3, info.actuatedDofNum);
  }

  matrix_t R_taskspace(totalContactDim + totalContactDim, totalContactDim + totalContactDim);
  loadData::LoadEigenMatrix(taskFile, "R", R_taskspace);

  matrix_t R = matrix_t::Zero(info.inputDim, info.inputDim);
  // Contact Forces
  R.topLeftCorner(totalContactDim, totalContactDim) = R_taskspace.topLeftCorner(totalContactDim, totalContactDim);
  // Joint velocities
  R.bottomRightCorner(info.actuatedDofNum, info.actuatedDofNum) =
      baseToFeetJacobians.transpose() * R_taskspace.bottomRightCorner(totalContactDim, totalContactDim) * baseToFeetJacobians;
  return R;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::GetBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info,
                                                                          bool verbose) {
  matrix_t Q(info.stateDim, info.stateDim);
  loadData::LoadEigenMatrix(taskFile, "Q", Q);
  matrix_t R = InitializeInputCostWeight(taskFile, info);

  if (verbose) {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::make_unique<LeggedRobotStateInputQuadraticCost>(std::move(Q), std::move(R), info, *referenceManagerPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config> LeggedRobotInterface::LoadFrictionConeSettings(const std::string& taskFile,
                                                                                                  bool verbose) const {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(taskFile, pt);
  const std::string prefix = "frictionConeSoftConstraint.";

  scalar_t frictionCoefficient = 1.0;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::LoadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
  loadData::LoadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::LoadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return {frictionCoefficient, std::move(barrierPenaltyConfig)};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::GetFrictionConeConstraint(size_t contactPointIndex,
                                                                                      scalar_t frictionCoefficient) {
  FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
  return std::make_unique<FrictionConeConstraint>(*referenceManagerPtr_, std::move(frictionConeConConfig), contactPointIndex,
                                                  centroidalModelInfo_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::GetFrictionConeSoftConstraint(
    size_t contactPointIndex, scalar_t frictionCoefficient, const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  return std::make_unique<StateInputSoftConstraint>(GetFrictionConeConstraint(contactPointIndex, frictionCoefficient),
                                                    std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfig));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::GetZeroForceConstraint(size_t contactPointIndex) {
  return std::make_unique<ZeroForceConstraint>(*referenceManagerPtr_, contactPointIndex, centroidalModelInfo_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::GetZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                      size_t contactPointIndex,
                                                                                      bool useAnalyticalGradients) {
  auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
    EndEffectorLinearConstraint::Config config;
    config.b.setZero(3);
    config.Av.setIdentity(3, 3);
    if (!numerics::almost_eq(positionErrorGain, 0.0)) {
      config.Ax.setZero(3, 3);
      config.Ax(2, 2) = positionErrorGain;
    }
    return config;
  };

  if (useAnalyticalGradients) {
    throw std::runtime_error(
        "[LeggedRobotInterface::GetZeroVelocityConstraint] The analytical end-effector zero velocity constraint is not implemented!");
  } else {
    return std::make_unique<ZeroVelocityConstraintCppAd>(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                                         eeZeroVelConConfig(modelSettings_.positionErrorGain));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::GetNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                        size_t contactPointIndex,
                                                                                        bool useAnalyticalGradients) {
  if (useAnalyticalGradients) {
    throw std::runtime_error(
        "[LeggedRobotInterface::GetNormalVelocityConstraint] The analytical end-effector normal velocity constraint is not implemented!");
  } else {
    return std::make_unique<NormalVelocityConstraintCppAd>(*referenceManagerPtr_, eeKinematics, contactPointIndex);
  }
}

}  // namespace legged_robot
}  // namespace bolza
