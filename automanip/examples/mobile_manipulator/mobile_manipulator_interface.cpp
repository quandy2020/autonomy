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
#include <filesystem>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include "examples/mobile_manipulator/mobile_manipulator_interface.hpp"

#include <bolza/core/initialization/default_initializer.hpp>
#include <bolza/core/misc/load_data.hpp>
#include <bolza/core/misc/load_std_vector_of_pair.hpp>
#include <bolza/core/penalties/penalties.hpp>
#include <bolza/core/soft_constraint/state_input_soft_box_constraint.hpp>
#include <bolza/core/soft_constraint/state_soft_constraint.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>
#include <bolza/pinocchio/interface/pinocchio_end_effector_kinematics.hpp>
#include <bolza/pinocchio/interface/pinocchio_end_effector_kinematics_cpp_ad.hpp>
#include <bolza/pinocchio/interface/urdf.hpp>
#include <bolza/pinocchio/self_collision/self_collision_constraint.hpp>
#include <bolza/pinocchio/self_collision/self_collision_constraint_cpp_ad.hpp>

#include "examples/mobile_manipulator/manipulator_model_info.hpp"
#include "examples/mobile_manipulator/mobile_manipulator_pre_computation.hpp"
#include "examples/mobile_manipulator/constraint/end_effector_constraint.hpp"
#include "examples/mobile_manipulator/constraint/mobile_manipulator_self_collision_constraint.hpp"
#include "examples/mobile_manipulator/cost/quadratic_input_cost.hpp"
#include "examples/mobile_manipulator/dynamics/default_manipulator_dynamics.hpp"
#include "examples/mobile_manipulator/dynamics/floating_arm_manipulator_dynamics.hpp"
#include "examples/mobile_manipulator/dynamics/fully_actuated_floating_arm_manipulator_dynamics.hpp"
#include "examples/mobile_manipulator/dynamics/wheel_based_mobile_manipulator_dynamics.hpp"

// Boost

namespace bolza {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorInterface::MobileManipulatorInterface(const std::string& taskFile, const std::string& libraryFolder,
                                                       const std::string& urdfFile) {
  // check that task file exists
  std::filesystem::path taskFilePath(taskFile);
  if (std::filesystem::exists(taskFilePath)) {
    std::cerr << "[MobileManipulatorInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[MobileManipulatorInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  std::filesystem::path urdfFilePath(urdfFile);
  if (std::filesystem::exists(urdfFilePath)) {
    std::cerr << "[MobileManipulatorInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[MobileManipulatorInterface] URDF file not found: " + urdfFilePath.string());
  }
  // create library folder if it does not exist
  std::filesystem::path libraryFolderPath(libraryFolder);
  std::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

  // read the task file
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(taskFile, pt);
  // resolve meta-information about the model
  // read manipulator type
  ManipulatorModelType modelType = mobile_manipulator::LoadManipulatorType(taskFile, "model_information.manipulatorModelType");
  // read the joints to make fixed
  std::vector<std::string> removeJointNames;
  loadData::LoadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
  // read the frame names
  std::string baseFrame, eeFrame;
  loadData::LoadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
  loadData::LoadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);

  std::cerr << "\n #### Model Information:";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "\n #### model_information.manipulatorModelType: " << static_cast<int>(modelType);
  std::cerr << "\n #### model_information.removeJoints: ";
  for (const auto& name : removeJointNames) {
    std::cerr << "\"" << Name << "\" ";
  }
  std::cerr << "\n #### model_information.baseFrame: \"" << baseFrame << "\"";
  std::cerr << "\n #### model_information.eeFrame: \"" << eeFrame << "\"" << std::endl;
  std::cerr << " #### =============================================================================" << std::endl;

  // create pinocchio interface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(CreatePinocchioInterface(urdfFile, modelType, removeJointNames)));
  std::cerr << *pinocchioInterfacePtr_;

  // ManipulatorModelInfo
  manipulatorModelInfo_ = mobile_manipulator::CreateManipulatorModelInfo(*pinocchioInterfacePtr_, modelType, baseFrame, eeFrame);

  bool usePreComputation = true;
  bool recompileLibraries = true;
  std::cerr << "\n #### Model Settings:";
  std::cerr << "\n #### =============================================================================\n";
  loadData::LoadPtreeValue(pt, usePreComputation, "model_settings.usePreComputation", true);
  loadData::LoadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
  std::cerr << " #### =============================================================================\n";

  // Default initial state
  initialState_.setZero(manipulatorModelInfo_.stateDim);
  const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
  const int armStateDim = manipulatorModelInfo_.armDim;

  // arm base DOFs initial state
  if (baseStateDim > 0) {
    vector_t initialBaseState = vector_t::Zero(baseStateDim);
    loadData::LoadEigenMatrix(taskFile, "initialState.base." + ModelTypeEnumToString(modelType), initialBaseState);
    initialState_.head(baseStateDim) = initialBaseState;
  }

  // arm joints DOFs velocity limits
  vector_t initialArmState = vector_t::Zero(armStateDim);
  loadData::LoadEigenMatrix(taskFile, "initialState.arm", initialArmState);
  initialState_.tail(armStateDim) = initialArmState;

  std::cerr << "Initial State:   " << initialState_.transpose() << std::endl;

  // DDP-MPC settings
  ddpSettings_ = ddp::LoadSettings(taskFile, "ddp");
  mpcSettings_ = mpc::LoadSettings(taskFile, "mpc");

  // Reference Manager
  referenceManagerPtr_.reset(new ReferenceManager);

  /*
   * Optimal control problem
   */
  // Cost
  problem_.costPtr->Add("inputCost", GetQuadraticInputCost(taskFile));

  // Constraints
  // joint limits constraint
  problem_.softConstraintPtr->Add("jointLimits", GetJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile));
  // end-effector state constraint
  problem_.stateSoftConstraintPtr->Add("endEffector", GetEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector",
                                                                               usePreComputation, libraryFolder, recompileLibraries));
  problem_.finalSoftConstraintPtr->Add("finalEndEffector", GetEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndEffector",
                                                                                    usePreComputation, libraryFolder, recompileLibraries));
  // self-collision avoidance constraint
  bool activateSelfCollision = true;
  loadData::LoadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
  if (activateSelfCollision) {
    problem_.stateSoftConstraintPtr->Add(
        "selfCollision", GetSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, urdfFile, "selfCollision", usePreComputation,
                                                    libraryFolder, recompileLibraries));
  }

  // Dynamics
  switch (manipulatorModelInfo_.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      problem_.dynamicsPtr.reset(
          new DefaultManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      problem_.dynamicsPtr.reset(
          new FloatingArmManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      problem_.dynamicsPtr.reset(
          new FullyActuatedFloatingArmManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      problem_.dynamicsPtr.reset(
          new WheelBasedMobileManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }

  /*
   * Pre-computation
   */
  if (usePreComputation) {
    problem_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, manipulatorModelInfo_));
  }

  // Rollout
  const auto rolloutSettings = rollout::LoadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Initialization
  initializerPtr_.reset(new DefaultInitializer(manipulatorModelInfo_.inputDim));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::GetQuadraticInputCost(const std::string& taskFile) {
  matrix_t R = matrix_t::Zero(manipulatorModelInfo_.inputDim, manipulatorModelInfo_.inputDim);
  const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
  const int armStateDim = manipulatorModelInfo_.armDim;

  // arm base DOFs input costs
  if (baseInputDim > 0) {
    matrix_t R_base = matrix_t::Zero(baseInputDim, baseInputDim);
    loadData::LoadEigenMatrix(taskFile, "inputCost.R.base." + ModelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType), R_base);
    R.topLeftCorner(baseInputDim, baseInputDim) = R_base;
  }

  // arm joints DOFs input costs
  matrix_t R_arm = matrix_t::Zero(armStateDim, armStateDim);
  loadData::LoadEigenMatrix(taskFile, "inputCost.R.arm", R_arm);
  R.bottomRightCorner(armStateDim, armStateDim) = R_arm;

  std::cerr << "\n #### Input Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "inputCost.R:  \n" << R << '\n';
  std::cerr << " #### =============================================================================\n";

  return std::make_unique<QuadraticInputCost>(std::move(R), manipulatorModelInfo_.stateDim);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::GetEndEffectorConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                const std::string& taskFile, const std::string& prefix,
                                                                                bool usePreComputation, const std::string& libraryFolder,
                                                                                bool recompileLibraries) {
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  const std::string name = "WRIST_2";

  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::LoadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  loadData::LoadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.eeFrame});
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  } else {
    MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {manipulatorModelInfo_.eeFrame},
                                                     manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                     "end_effector_kinematics", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> MobileManipulatorInterface::GetSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                  const std::string& taskFile, const std::string& urdfFile,
                                                                                  const std::string& prefix, bool usePreComputation,
                                                                                  const std::string& libraryFolder,
                                                                                  bool recompileLibraries) {
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.0;

  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### SelfCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::LoadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::LoadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::LoadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
  loadData::LoadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
  loadData::LoadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
  std::cerr << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);

  const size_t numCollisionPairs = geometryInterface.GetNumCollisionPairs();
  std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    constraint = std::make_unique<MobileManipulatorSelfCollisionConstraint>(MobileManipulatorPinocchioMapping(manipulatorModelInfo_),
                                                                            std::move(geometryInterface), minimumDistance);
  } else {
    constraint = std::make_unique<SelfCollisionConstraintCppAd>(
        pinocchioInterface, MobileManipulatorPinocchioMapping(manipulatorModelInfo_), std::move(geometryInterface), minimumDistance,
        "self_collision", libraryFolder, recompileLibraries, false);
  }

  auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> MobileManipulatorInterface::GetJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                        const std::string& taskFile) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(taskFile, pt);

  bool activateJointPositionLimit = true;
  loadData::LoadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

  const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
  const int armStateDim = manipulatorModelInfo_.armDim;
  const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
  const int armInputDim = manipulatorModelInfo_.armDim;
  const auto& model = pinocchioInterface.GetModel();

  // Load position limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit) {
    scalar_t muPositionLimits = 1e-2;
    scalar_t deltaPositionLimits = 1e-3;

    // arm joint DOF limits from the parsed URDF
    const vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
    const vector_t upperBound = model.upperPositionLimit.tail(armStateDim);

    std::cerr << "\n #### JointPositionLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
    std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
    loadData::LoadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
    loadData::LoadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    stateLimits.reserve(armStateDim);
    for (int i = 0; i < armStateDim; ++i) {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = baseStateDim + i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
      stateLimits.push_back(std::move(boxConstraint));
    }
  }

  // load velocity limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
  {
    vector_t lowerBound = vector_t::Zero(manipulatorModelInfo_.inputDim);
    vector_t upperBound = vector_t::Zero(manipulatorModelInfo_.inputDim);
    scalar_t muVelocityLimits = 1e-2;
    scalar_t deltaVelocityLimits = 1e-3;

    // Base DOFs velocity limits
    if (baseInputDim > 0) {
      vector_t lowerBoundBase = vector_t::Zero(baseInputDim);
      vector_t upperBoundBase = vector_t::Zero(baseInputDim);
      loadData::LoadEigenMatrix(taskFile,
                                "jointVelocityLimits.lowerBound.base." + ModelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                                lowerBoundBase);
      loadData::LoadEigenMatrix(taskFile,
                                "jointVelocityLimits.upperBound.base." + ModelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                                upperBoundBase);
      lowerBound.head(baseInputDim) = lowerBoundBase;
      upperBound.head(baseInputDim) = upperBoundBase;
    }

    // arm joint DOFs velocity limits
    vector_t lowerBoundArm = vector_t::Zero(armInputDim);
    vector_t upperBoundArm = vector_t::Zero(armInputDim);
    loadData::LoadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
    loadData::LoadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);
    lowerBound.tail(armInputDim) = lowerBoundArm;
    upperBound.tail(armInputDim) = upperBoundArm;

    std::cerr << "\n #### JointVelocityLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
    std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
    loadData::LoadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
    loadData::LoadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    inputLimits.reserve(manipulatorModelInfo_.inputDim);
    for (int i = 0; i < manipulatorModelInfo_.inputDim; ++i) {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
      inputLimits.push_back(std::move(boxConstraint));
    }
  }

  auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
  boxConstraints->InitializeOffset(0.0, vector_t::Zero(manipulatorModelInfo_.stateDim), vector_t::Zero(manipulatorModelInfo_.inputDim));
  return boxConstraints;
}

}  // namespace mobile_manipulator
}  // namespace bolza
