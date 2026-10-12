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

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <bolza/tools/common/angular_velocity_mapping.hpp>
#include <bolza/tools/common/rotation_transforms.hpp>
#include <bolza/tools/common/skew_symmetric_matrix.hpp>

#include "bolza/pinocchio/interface/pinocchio_end_effector_kinematics.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics::PinocchioEndEffectorKinematics(const PinocchioInterface& pinocchioInterface,
                                                               const PinocchioStateInputMapping<scalar_t>& mapping,
                                                               std::vector<std::string> endEffectorIds)
    : pinocchioInterfacePtr_(nullptr), mappingPtr_(mapping.clone()), endEffectorIds_(std::move(endEffectorIds)) {
  for (const auto& bodyName : endEffectorIds_) {
    endEffectorFrameIds_.push_back(pinocchioInterface.getModel().getBodyId(bodyName));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics::PinocchioEndEffectorKinematics(const PinocchioEndEffectorKinematics& rhs)
    : EndEffectorKinematics<scalar_t>(rhs),
      pinocchioInterfacePtr_(nullptr),
      mappingPtr_(rhs.mappingPtr_->clone()),
      endEffectorIds_(rhs.endEffectorIds_),
      endEffectorFrameIds_(rhs.endEffectorFrameIds_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics* PinocchioEndEffectorKinematics::Clone() const {
  return new PinocchioEndEffectorKinematics(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const std::vector<std::string>& PinocchioEndEffectorKinematics::GetIds() const {
  return endEffectorIds_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::GetPosition(const vector_t& state) const -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use SetPinocchioInterface()");
  }

  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> positions;
  for (const auto& frameId : endEffectorFrameIds_) {
    positions.emplace_back(data.oMf[frameId].translation());
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::GetVelocity(const vector_t& state, const vector_t& input) const -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use SetPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> velocities;
  for (const auto& frameId : endEffectorFrameIds_) {
    velocities.emplace_back(pinocchio::getFrameVelocity(model, data, frameId, rf).linear());
  }
  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::GetPositionLinearApproximation(const vector_t& state) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use SetPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> positions;
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t J = matrix_t::Zero(6, model.nq);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);

    VectorFunctionLinearApproximation pos;
    pos.f = data.oMf[frameId].translation();
    std::tie(pos.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, J.topRows<3>(), matrix_t::Zero(3, model.nv));
    positions.emplace_back(std::move(pos));
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::GetVelocityLinearApproximation(const vector_t& state,
                                                                                                              const vector_t& input) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use SetPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> velocities;
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t v_partial_dq = matrix_t::Zero(6, model.nv);
    matrix_t v_partial_dv = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameVelocityDerivatives(model, data, frameId, rf, v_partial_dq, v_partial_dv);
    const auto frameVel = pinocchio::getFrameVelocity(model, data, frameId, rf);
    // For reference frame LOCAL_WORLD_ALIGNED the jacobian needs to be corrected.
    if (rf == pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED) {
      v_partial_dq.topRows<3>() += SkewSymmetricMatrix(vector3_t(frameVel.angular())) * v_partial_dv.topRows<3>();
    }
    VectorFunctionLinearApproximation vel;
    vel.f = frameVel.linear();
    std::tie(vel.dfdx, vel.dfdu) = mappingPtr_->getOcs2Jacobian(state, v_partial_dq.topRows<3>(), v_partial_dv.topRows<3>());
    velocities.emplace_back(std::move(vel));
  }
  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::GetOrientationError(const vector_t& state,
                                                         const std::vector<quaternion_t>& referenceOrientations) const
    -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use SetPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> errors;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    const size_t frameId = endEffectorFrameIds_[i];
    errors.emplace_back(QuaternionDistance(MatrixToQuaternion(data.oMf[frameId].rotation()), referenceOrientations[i]));
  }
  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::GetOrientationErrorLinearApproximation(
    const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use SetPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> errors;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    VectorFunctionLinearApproximation err;
    const size_t frameId = endEffectorFrameIds_[i];
    const quaternion_t q = MatrixToQuaternion(data.oMf[frameId].rotation());
    err.f = QuaternionDistance(q, referenceOrientations[i]);
    matrix_t J = matrix_t::Zero(6, model.nq);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);
    const matrix_t Jqdist =
        (QuaternionDistanceJacobian(q, referenceOrientations[i]) * AngularVelocityToQuaternionTimeDerivative(q)) * J.bottomRows<3>();
    std::tie(err.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, Jqdist, matrix_t::Zero(3, model.nv));
    errors.emplace_back(std::move(err));
  }
  return errors;
}

}  // namespace bolza
