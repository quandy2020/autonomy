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

#include "bolza/pinocchio/interface/pinocchio_end_effector_kinematics_cpp_ad.hpp"
#include <bolza/tools/common/rotation_transforms.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace {

void defaultUpdatePinocchioInterface(const bolza::ad_vector_t&, bolza::PinocchioInterfaceTpl<bolza::ad_scalar_t>&) {}

}  // unnamed namespace

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd(const PinocchioInterface& pinocchioInterface,
                                                                         const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                         std::vector<std::string> endEffectorIds, size_t stateDim,
                                                                         size_t inputDim, const std::string& modelName,
                                                                         const std::string& modelFolder, bool recompileLibraries,
                                                                         bool verbose)

    : PinocchioEndEffectorKinematicsCppAd(pinocchioInterface, mapping, std::move(endEffectorIds), stateDim, inputDim,
                                          &defaultUpdatePinocchioInterface, modelName, modelFolder, recompileLibraries, verbose) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd(
    const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
    std::vector<std::string> endEffectorIds, size_t stateDim, size_t inputDim, update_pinocchio_interface_callback updateCallback,
    const std::string& modelName, const std::string& modelFolder, bool recompileLibraries, bool verbose)
    : endEffectorIds_(std::move(endEffectorIds)) {
  for (const auto& bodyName : endEffectorIds_) {
    endEffectorFrameIds_.push_back(pinocchioInterface.getModel().getBodyId(bodyName));
  }

  // initialize CppAD interface
  auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

  // set pinocchioInterface to mapping
  std::unique_ptr<PinocchioStateInputMapping<ad_scalar_t>> mappingPtr(mapping.clone());
  mappingPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

  // position function
  auto positionFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {
    updateCallback(x, pinocchioInterfaceCppAd);
    y = GetPositionCppAd(pinocchioInterfaceCppAd, *mappingPtr, x);
  };
  positionCppAdInterfacePtr_.reset(new CppAdInterface(positionFunc, stateDim, modelName + "_position", modelFolder));

  // velocity function
  auto velocityFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {
    const ad_vector_t state = x.head(stateDim);
    const ad_vector_t input = x.tail(inputDim);
    updateCallback(state, pinocchioInterfaceCppAd);
    y = GetVelocityCppAd(pinocchioInterfaceCppAd, *mappingPtr, state, input);
  };
  velocityCppAdInterfacePtr_.reset(new CppAdInterface(velocityFunc, stateDim + inputDim, modelName + "_velocity", modelFolder));

  // orientation function
  auto orientationFunc = [&, this](const ad_vector_t& x, const ad_vector_t& params, ad_vector_t& y) {
    updateCallback(x, pinocchioInterfaceCppAd);
    y = GetOrientationErrorCppAd(pinocchioInterfaceCppAd, *mappingPtr, x, params);
  };
  orientationErrorCppAdInterfacePtr_.reset(
      new CppAdInterface(orientationFunc, stateDim, 4 * endEffectorFrameIds_.size(), modelName + "_orientation", modelFolder));

  if (recompileLibraries) {
    positionCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    velocityCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    orientationErrorCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    positionCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    velocityCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    orientationErrorCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematicsCppAd::PinocchioEndEffectorKinematicsCppAd(const PinocchioEndEffectorKinematicsCppAd& rhs)
    : EndEffectorKinematics<scalar_t>(rhs),
      positionCppAdInterfacePtr_(new CppAdInterface(*rhs.positionCppAdInterfacePtr_)),
      velocityCppAdInterfacePtr_(new CppAdInterface(*rhs.velocityCppAdInterfacePtr_)),
      orientationErrorCppAdInterfacePtr_(new CppAdInterface(*rhs.orientationErrorCppAdInterfacePtr_)),
      endEffectorIds_(rhs.endEffectorIds_),
      endEffectorFrameIds_(rhs.endEffectorFrameIds_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematicsCppAd* PinocchioEndEffectorKinematicsCppAd::Clone() const {
  return new PinocchioEndEffectorKinematicsCppAd(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const std::vector<std::string>& PinocchioEndEffectorKinematicsCppAd::GetIds() const {
  return endEffectorIds_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioEndEffectorKinematicsCppAd::GetPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                  const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                  const ad_vector_t& state) {
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mapping.getPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  ad_vector_t positions(3 * endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    const size_t frameId = endEffectorFrameIds_[i];
    positions.segment<3>(3 * i) = data.oMf[frameId].translation();
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematicsCppAd::GetPosition(const vector_t& state) const -> std::vector<vector3_t> {
  const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);

  std::vector<vector3_t> positions;
  for (int i = 0; i < endEffectorIds_.size(); i++) {
    positions.emplace_back(positionValues.segment<3>(3 * i));
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::GetPositionLinearApproximation(
    const vector_t& state) const {
  const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);
  const matrix_t positionJacobian = positionCppAdInterfacePtr_->getJacobian(state);

  std::vector<VectorFunctionLinearApproximation> positions;
  for (int i = 0; i < endEffectorIds_.size(); i++) {
    VectorFunctionLinearApproximation pos;
    pos.f = positionValues.segment<3>(3 * i);
    pos.dfdx = positionJacobian.block(3 * i, 0, 3, state.rows());
    positions.emplace_back(std::move(pos));
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioEndEffectorKinematicsCppAd::GetVelocityCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                  const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                  const ad_vector_t& state, const ad_vector_t& input) {
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mapping.getPinocchioJointPosition(state);
  const ad_vector_t v = mapping.getPinocchioJointVelocity(state, input);

  pinocchio::forwardKinematics(model, data, q, v);

  ad_vector_t velocities(3 * endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    const size_t frameId = endEffectorFrameIds_[i];
    velocities.segment<3>(3 * i) = pinocchio::getFrameVelocity(model, data, frameId, rf).linear();
  }
  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematicsCppAd::GetVelocity(const vector_t& state, const vector_t& input) const -> std::vector<vector3_t> {
  vector_t stateInput(state.rows() + input.rows());
  stateInput << state, input;
  const vector_t velocityValues = velocityCppAdInterfacePtr_->getFunctionValue(stateInput);

  std::vector<vector3_t> velocities;
  for (int i = 0; i < endEffectorIds_.size(); i++) {
    velocities.emplace_back(velocityValues.segment<3>(3 * i));
  }
  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::GetVelocityLinearApproximation(
    const vector_t& state, const vector_t& input) const {
  vector_t stateInput(state.rows() + input.rows());
  stateInput << state, input;
  const vector_t velocityValues = velocityCppAdInterfacePtr_->getFunctionValue(stateInput);
  const matrix_t velocityJacobian = velocityCppAdInterfacePtr_->getJacobian(stateInput);

  std::vector<VectorFunctionLinearApproximation> velocities;
  for (int i = 0; i < endEffectorIds_.size(); i++) {
    VectorFunctionLinearApproximation vel;
    vel.f = velocityValues.segment<3>(3 * i);
    vel.dfdx = velocityJacobian.block(3 * i, 0, 3, state.rows());
    vel.dfdu = velocityJacobian.block(3 * i, state.rows(), 3, input.rows());
    velocities.emplace_back(std::move(vel));
  }
  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematicsCppAd::GetOrientationError(const vector_t& state,
                                                              const std::vector<quaternion_t>& referenceOrientations) const
    -> std::vector<vector3_t> {
  vector_t params(4 * endEffectorIds_.size());
  for (int i = 0; i < endEffectorIds_.size(); i++) {
    params.segment<4>(4 * i) = referenceOrientations[i].coeffs();
  }

  const vector_t errorValues = orientationErrorCppAdInterfacePtr_->getFunctionValue(state, params);

  std::vector<vector3_t> errors;
  for (int i = 0; i < endEffectorIds_.size(); i++) {
    errors.emplace_back(errorValues.segment<3>(3 * i));
  }
  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematicsCppAd::GetOrientationErrorLinearApproximation(
    const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const {
  vector_t params(4 * endEffectorIds_.size());
  for (int i = 0; i < endEffectorIds_.size(); i++) {
    params.segment<4>(4 * i) = referenceOrientations[i].coeffs();
  }

  const vector_t errorValues = orientationErrorCppAdInterfacePtr_->getFunctionValue(state, params);
  const matrix_t errorJacobian = orientationErrorCppAdInterfacePtr_->getJacobian(state, params);

  std::vector<VectorFunctionLinearApproximation> errors;
  for (int i = 0; i < endEffectorIds_.size(); i++) {
    VectorFunctionLinearApproximation err;
    err.f = errorValues.segment<3>(3 * i);
    err.dfdx = errorJacobian.block(3 * i, 0, 3, state.rows());
    errors.emplace_back(std::move(err));
  }
  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioEndEffectorKinematicsCppAd::GetOrientationErrorCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                                          const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                          const ad_vector_t& state, const ad_vector_t& params) {
  using ad_quaternion_t = Eigen::Quaternion<ad_scalar_t>;

  const auto& model = pinocchioInterfaceCppAd.getModel();
  auto& data = pinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mapping.getPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  ad_vector_t errors(3 * endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    const size_t frameId = endEffectorFrameIds_[i];
    const ad_quaternion_t eeOrientation = MatrixToQuaternion(data.oMf[frameId].rotation());
    ad_quaternion_t eeReferenceOrientation;
    eeReferenceOrientation.coeffs() = params.segment<4>(4 * i);
    errors.segment<3>(3 * i) = bolza::QuaternionDistance(eeOrientation, eeReferenceOrientation);
  }
  return errors;
}

}  // namespace bolza
