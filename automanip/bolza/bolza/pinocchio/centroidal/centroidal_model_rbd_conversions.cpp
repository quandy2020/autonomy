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

#include "bolza/pinocchio/centroidal/centroidal_model_rbd_conversions.hpp"

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <bolza/core/misc/load_data.hpp>
#include <bolza/tools/common/rotation_derivatives_transforms.hpp>

#include "bolza/pinocchio/centroidal/access_helper_functions.hpp"
#include "bolza/pinocchio/centroidal/model_helper_functions.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelRbdConversions::CentroidalModelRbdConversions(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& info)
    : pinocchioInterface_(std::move(pinocchioInterface)), mapping_(info) {
  mapping_.setPinocchioInterface(pinocchioInterface_);
  pGains_ = vector_t::Zero(mapping_.getCentroidalModelInfo().generalizedCoordinatesNum);
  dGains_ = vector_t::Zero(mapping_.getCentroidalModelInfo().generalizedCoordinatesNum);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::ComputeBaseKinematicsFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                             const vector_t& jointAccelerations, Vector6& basePose,
                                                                             Vector6& baseVelocity, Vector6& baseAcceleration) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_.getCentroidalModelInfo();
  const auto qPinocchio = mapping_.getPinocchioJointPosition(state);

  UpdateCentroidalDynamics(pinocchioInterface_, info, qPinocchio);

  // Base Pose in world frame
  basePose = qPinocchio.head<6>();
  const auto basePosition = basePose.head<3>();
  const auto baseOrientation = basePose.tail<3>();

  // Base Velocity in world frame
  const auto& A = GetCentroidalMomentumMatrix(pinocchioInterface_);
  const Matrix6 Ab = A.template leftCols<6>();
  const auto Ab_inv = ComputeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info.actuatedDofNum);

  const vector_t vPinocchio = mapping_.getPinocchioJointVelocity(state, input);
  baseVelocity.head<3>() = vPinocchio.head<3>();
  const Vector3 derivativeEulerAnglesZyx = vPinocchio.segment<3>(3);
  baseVelocity.tail<3>() = GetGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(baseOrientation, derivativeEulerAnglesZyx);

  const auto Adot = pinocchio::dccrba(model, data, qPinocchio, vPinocchio);
  Vector6 centroidalMomentumRate = info.robotMass * GetNormalizedCentroidalMomentumRate(pinocchioInterface_, info, input);
  centroidalMomentumRate.noalias() -= Adot * vPinocchio;
  centroidalMomentumRate.noalias() -= Aj * jointAccelerations.head(info.actuatedDofNum);
  const Vector6 qbaseDdot = Ab_inv * centroidalMomentumRate;

  // Base Acceleration in world frame
  baseAcceleration.head<3>() = qbaseDdot.head<3>();
  baseAcceleration.tail<3>() =
      GetGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(baseOrientation, derivativeEulerAnglesZyx, qbaseDdot.tail<3>());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CentroidalModelRbdConversions::ComputeCentroidalStateFromRbdModel(const vector_t& rbdState) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_.getCentroidalModelInfo();

  vector_t qPinocchio(info.generalizedCoordinatesNum);
  qPinocchio.head<3>() = rbdState.segment<3>(3);
  qPinocchio.segment<3>(3) = rbdState.head<3>();
  qPinocchio.tail(info.actuatedDofNum) = rbdState.segment(6, info.actuatedDofNum);

  vector_t vPinocchio(info.generalizedCoordinatesNum);
  vPinocchio.head<3>() = rbdState.segment<3>(info.generalizedCoordinatesNum + 3);
  vPinocchio.segment<3>(3) = GetEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPinocchio.segment<3>(3), rbdState.segment<3>(info.generalizedCoordinatesNum));
  vPinocchio.tail(info.actuatedDofNum) = rbdState.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum);

  UpdateCentroidalDynamics(pinocchioInterface_, info, qPinocchio);
  const auto& A = GetCentroidalMomentumMatrix(pinocchioInterface_);

  vector_t state(info.stateDim);
  centroidal_model::GetNormalizedMomentum(state, info).noalias() = A * vPinocchio / info.robotMass;
  centroidal_model::getGeneralizedCoordinates(state, info) = qPinocchio;

  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CentroidalModelRbdConversions::ComputeRbdStateFromCentroidalModel(const vector_t& state, const vector_t& input) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_.getCentroidalModelInfo();
  const vector_t jointAccelerations = vector_t::Zero(info.actuatedDofNum);

  Vector6 basePose, baseVelocity, baseAcceleration;
  ComputeBaseKinematicsFromCentroidalModel(state, input, jointAccelerations, basePose, baseVelocity, baseAcceleration);

  vector_t rbdState(2 * info.generalizedCoordinatesNum);
  rbdState.segment<3>(0) = basePose.tail<3>();
  rbdState.segment<3>(3) = basePose.head<3>();
  rbdState.segment(6, info.actuatedDofNum) = centroidal_model::getJointAngles(state, info);
  rbdState.segment<3>(info.generalizedCoordinatesNum) = baseVelocity.tail<3>();
  rbdState.segment<3>(info.generalizedCoordinatesNum + 3) = baseVelocity.head<3>();
  rbdState.tail(info.actuatedDofNum) = centroidal_model::GetJointVelocities(input, info);
  return rbdState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CentroidalModelRbdConversions::ComputeRbdTorqueFromCentroidalModel(const vector_t& state, const vector_t& input,
                                                                            const vector_t& jointAccelerations) {
  const auto& info = mapping_.getCentroidalModelInfo();
  const vector_t measuredRbdState = vector_t::Zero(2 * info.generalizedCoordinatesNum);
  const vector_t pGains = vector_t::Zero(info.generalizedCoordinatesNum);
  const vector_t dGains = vector_t::Zero(info.generalizedCoordinatesNum);
  return ComputeRbdTorqueFromCentroidalModelPD(state, input, jointAccelerations, measuredRbdState, pGains, dGains);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CentroidalModelRbdConversions::ComputeRbdTorqueFromCentroidalModelPD(const vector_t& desiredState, const vector_t& desiredInput,
                                                                              const vector_t& desiredJointAccelerations,
                                                                              const vector_t& measuredRbdState) {
  return ComputeRbdTorqueFromCentroidalModelPD(desiredState, desiredInput, desiredJointAccelerations, measuredRbdState, pGains_, dGains_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CentroidalModelRbdConversions::ComputeRbdTorqueFromCentroidalModelPD(const vector_t& desiredState, const vector_t& desiredInput,
                                                                              const vector_t& desiredJointAccelerations,
                                                                              const vector_t& measuredRbdState, const vector_t& pGains,
                                                                              const vector_t& dGains) {
  // handles
  const auto& info = mapping_.getCentroidalModelInfo();
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  // desired
  Vector6 desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration;
  ComputeBaseKinematicsFromCentroidalModel(desiredState, desiredInput, desiredJointAccelerations, desiredBasePose, desiredBaseVelocity,
                                           desiredBaseAcceleration);
  vector_t qDesired(info.generalizedCoordinatesNum), vDesired(info.generalizedCoordinatesNum), aDesired(info.generalizedCoordinatesNum);
  qDesired << desiredBasePose, centroidal_model::getJointAngles(desiredState, info);
  vDesired << desiredBaseVelocity, centroidal_model::GetJointVelocities(desiredInput, info);
  aDesired << desiredBaseAcceleration, desiredJointAccelerations;

  pinocchio::container::aligned_vector<pinocchio::Force> fextDesired(model.njoints, pinocchio::Force::Zero());
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto frameIndex = info.endEffectorFrameIndices[i];
#if PINOCCHIO_VERSION_AT_LEAST(3, 0, 0)
    const auto jointIndex = model.frames[frameIndex].parentJoint;
#else
    const auto jointIndex = model.frames[frameIndex].parent;
#endif
    const Vector3 translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();
    const Matrix3 rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();
    const Vector3 contactForce = rotationWorldFrameToJointFrame * centroidal_model::GetContactForces(desiredInput, i, info);
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce);
  }
  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const auto frameIndex = info.endEffectorFrameIndices[i];
#if PINOCCHIO_VERSION_AT_LEAST(3, 0, 0)
    const auto jointIndex = model.frames[frameIndex].parentJoint;
#else
    const auto jointIndex = model.frames[frameIndex].parent;
#endif
    const Vector3 translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();
    const Matrix3 rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();
    const Vector3 contactForce = rotationWorldFrameToJointFrame * centroidal_model::GetContactForces(desiredInput, i, info);
    const Vector3 contactTorque = rotationWorldFrameToJointFrame * centroidal_model::GetContactTorques(desiredInput, i, info);
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce) + contactTorque;
  }

  // measured
  vector_t qMeasured(info.generalizedCoordinatesNum), vMeasured(info.generalizedCoordinatesNum);
  qMeasured.head<3>() = measuredRbdState.segment<3>(3);
  qMeasured.segment<3>(3) = measuredRbdState.head<3>();
  qMeasured.tail(info.actuatedDofNum) = measuredRbdState.segment(6, info.actuatedDofNum);
  vMeasured.head<3>() = measuredRbdState.segment<3>(info.generalizedCoordinatesNum + 3);
  vMeasured.segment<3>(3) = GetEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured.segment<3>(3), measuredRbdState.segment<3>(info.generalizedCoordinatesNum));
  vMeasured.tail(info.actuatedDofNum) = measuredRbdState.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum);

  // PD feedback augmentation
  const vector_t pdFeedback = pGains.cwiseProduct(qDesired - qMeasured) + dGains.cwiseProduct(vDesired - vMeasured);

  // feedforward plus PD on acceleration level
  const vector_t aAugmented = aDesired + pdFeedback;
  return pinocchio::rnea(model, data, qDesired, vDesired, aAugmented, fextDesired);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalModelRbdConversions::LoadSettings(const std::string& fileName, const std::string& fieldName, bool verbose) {
  if (verbose) {
    std::cerr << "\n#### CentroidalModelRbdConversionsSettings:\n";
    std::cerr << "#### =============================================================================" << std::endl;
  }

  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(fileName, pt);
  const std::string centroidalModelRbdConversionsFieldName = fieldName + ".centroidal_model_rbd_conversions";

  std::vector<scalar_t> pGainsVec, dGainsVec;
  loadData::LoadStdVector(fileName, centroidalModelRbdConversionsFieldName + ".pGains", pGainsVec, verbose);
  if (!pGainsVec.empty()) {
    pGains_ = Eigen::Map<vector_t>(pGainsVec.data(), pGainsVec.size());
  }
  loadData::LoadStdVector(fileName, centroidalModelRbdConversionsFieldName + ".dGains", dGainsVec, verbose);
  if (!dGainsVec.empty()) {
    dGains_ = Eigen::Map<vector_t>(dGainsVec.data(), dGainsVec.size());
  }
}

}  // namespace bolza
