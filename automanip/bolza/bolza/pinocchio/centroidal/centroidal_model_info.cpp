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

#include "bolza/pinocchio/centroidal/centroidal_model_info.hpp"

namespace bolza {

std::string ToString(CentroidalModelType type) {
  switch (type) {
    case CentroidalModelType::FullCentroidalDynamics:
      return "FullCentroidalDynamics";
    case CentroidalModelType::SingleRigidBodyDynamics:
      return "SingleRigidBodyDynamics";
    default:
      return "Undefined CentroidalModelType!";
  }
}

std::ostream& operator<<(std::ostream& os, CentroidalModelType type) {
  os << ToString(type);
  return os;
}

template <>
template <>
CentroidalModelInfoCppAd CentroidalModelInfo::ToCppAd() const {
  CentroidalModelInfoCppAd cppAdInfo;

  cppAdInfo.centroidalModelType = this->centroidalModelType;
  cppAdInfo.numThreeDofContacts = this->numThreeDofContacts;
  cppAdInfo.numSixDofContacts = this->numSixDofContacts;
  cppAdInfo.endEffectorFrameIndices = this->endEffectorFrameIndices;
  cppAdInfo.generalizedCoordinatesNum = this->generalizedCoordinatesNum;
  cppAdInfo.actuatedDofNum = this->actuatedDofNum;
  cppAdInfo.stateDim = this->stateDim;
  cppAdInfo.inputDim = this->inputDim;
  cppAdInfo.robotMass = ad_scalar_t(this->robotMass);
  cppAdInfo.qPinocchioNominal = this->qPinocchioNominal.cast<ad_scalar_t>();
  cppAdInfo.centroidalInertiaNominal = this->centroidalInertiaNominal.cast<ad_scalar_t>();
  cppAdInfo.comToBasePositionNominal = this->comToBasePositionNominal.cast<ad_scalar_t>();

  return cppAdInfo;
}

// explicit template instantiation
template struct bolza::CentroidalModelInfoTpl<bolza::scalar_t>;
template struct bolza::CentroidalModelInfoTpl<bolza::ad_scalar_t>;

}  // namespace bolza
