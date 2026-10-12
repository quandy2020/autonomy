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
#include <pinocchio/multibody/geometry.hpp>

#include <bolza/tools/common/angular_velocity_mapping.hpp>
#include <bolza/tools/common/rotation_transforms.hpp>
#include <bolza/tools/common/skew_symmetric_matrix.hpp>

#include "bolza/pinocchio/sphere/pinocchio_sphere_kinematics.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematics::PinocchioSphereKinematics(PinocchioSphereInterface pinocchioSphereInterface,
                                                     const PinocchioStateInputMapping<scalar_t>& mapping)
    : pinocchioInterfacePtr_(nullptr), pinocchioSphereInterface_(std::move(pinocchioSphereInterface)), mappingPtr_(mapping.clone()) {
  linkIds_.resize(pinocchioSphereInterface_.getNumSpheresInTotal());
  const std::vector<std::string>& collisionLinkOfEachPrimitiveShape = pinocchioSphereInterface_.getCollisionLinkOfEachPrimitveShape();
  const auto numSpheres = pinocchioSphereInterface_.getNumSpheres();
  size_t count = 0;
  for (size_t i = 0; i < pinocchioSphereInterface.getNumPrimitiveShapes(); i++) {
    std::fill(linkIds_.begin() + count, linkIds_.begin() + count + numSpheres[i], collisionLinkOfEachPrimitiveShape[i]);
    count += numSpheres[i];
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematics::PinocchioSphereKinematics(const PinocchioSphereKinematics& rhs)
    : EndEffectorKinematics<scalar_t>(rhs),
      pinocchioInterfacePtr_(nullptr),
      mappingPtr_(rhs.mappingPtr_->clone()),
      pinocchioSphereInterface_(rhs.pinocchioSphereInterface_),
      linkIds_(rhs.linkIds_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematics* PinocchioSphereKinematics::Clone() const {
  return new PinocchioSphereKinematics(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioSphereKinematics::GetPosition(const vector_t& state) const -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioSphereKinematics] pinocchioInterfacePtr_ is not set. Use SetPinocchioInterface()");
  }

  return pinocchioSphereInterface_.computeSphereCentersInWorldFrame(*pinocchioInterfacePtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioSphereKinematics::GetPositionLinearApproximation(const vector_t& state) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioSphereKinematics] pinocchioInterfacePtr_ is not set. Use SetPinocchioInterface()");
  }

  std::vector<vector3_t> sphereCentersInWorldFrame = pinocchioSphereInterface_.computeSphereCentersInWorldFrame(*pinocchioInterfacePtr_);

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> positions;

  const auto& geometryModel = pinocchioSphereInterface_.getGeometryModel();
  const size_t numPrimitiveShapes = pinocchioSphereInterface_.getNumPrimitiveShapes();
  const auto geomObjIds = pinocchioSphereInterface_.getGeomObjIds();
  const auto numSpheres = pinocchioSphereInterface_.getNumSpheres();

  size_t count = 0;
  for (size_t i = 0; i < numPrimitiveShapes; i++) {
    const auto& parentJointId = geometryModel.geometryObjects[geomObjIds[i]].parentJoint;
    const vector3_t& jointPosition = data.oMi[parentJointId].translation();
    matrix_t jointJacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getJointJacobian(model, data, parentJointId, rf, jointJacobian);

    for (size_t j = 0; j < numSpheres[i]; j++) {
      VectorFunctionLinearApproximation pos;
      pos.f = sphereCentersInWorldFrame[count + j];
      const vector3_t sphereCenterOffset = sphereCentersInWorldFrame[count + j] - jointPosition;
      const matrix_t sphereCenterJacobian =
          jointJacobian.topRows<3>() - SkewSymmetricMatrix(sphereCenterOffset) * jointJacobian.bottomRows<3>();
      std::tie(pos.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, sphereCenterJacobian.topRows<3>(), matrix_t::Zero(0, model.nq));
      positions.emplace_back(std::move(pos));
    }
    count += numSpheres[i];
  }

  return positions;
}

}  // namespace bolza
