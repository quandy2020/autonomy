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

#include <bolza/tools/common/rotation_transforms.hpp>
#include "bolza/pinocchio/sphere/pinocchio_sphere_kinematics_cpp_ad.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematicsCppAd::PinocchioSphereKinematicsCppAd(const PinocchioInterface& pinocchioInterface,
                                                               PinocchioSphereInterface pinocchioSphereInterface,
                                                               const PinocchioStateInputMapping<ad_scalar_t>& mapping, size_t stateDim,
                                                               size_t inputDim, const std::string& modelName,
                                                               const std::string& modelFolder, bool recompileLibraries, bool verbose)
    : pinocchioSphereInterface_(std::move(pinocchioSphereInterface)) {
  const std::vector<SphereApproxParam> sphereApproxParams = CreateSphereApproxParams();

  linkIds_.resize(pinocchioSphereInterface_.GetNumSpheresInTotal());
  const std::vector<std::string>& collisionLinkOfEachPrimitiveShape = pinocchioSphereInterface_.GetCollisionLinkOfEachPrimitveShape();
  const auto numSpheres = pinocchioSphereInterface_.GetNumSpheres();
  size_t count = 0;
  for (size_t i = 0; i < pinocchioSphereInterface.GetNumPrimitiveShapes(); i++) {
    std::fill(linkIds_.begin() + count, linkIds_.begin() + count + numSpheres[i], collisionLinkOfEachPrimitiveShape[i]);
    count += numSpheres[i];
  }

  // initialize CppAD interface
  auto pinocchioInterfaceCppAd = pinocchioInterface.ToCppAd();

  // position function
  auto positionFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {
    y = GetPositionCppAd(pinocchioInterfaceCppAd, mapping, sphereApproxParams, x);
  };
  positionCppAdInterfacePtr_.reset(new CppAdInterface(positionFunc, stateDim, modelName + "_position", modelFolder));

  if (recompileLibraries) {
    positionCppAdInterfacePtr_->CreateModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    positionCppAdInterfacePtr_->LoadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematicsCppAd::PinocchioSphereKinematicsCppAd(const PinocchioSphereKinematicsCppAd& rhs)
    : EndEffectorKinematics<scalar_t>(rhs),
      positionCppAdInterfacePtr_(new CppAdInterface(*rhs.positionCppAdInterfacePtr_)),
      pinocchioSphereInterface_(rhs.pinocchioSphereInterface_),
      linkIds_(rhs.linkIds_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematicsCppAd* PinocchioSphereKinematicsCppAd::Clone() const {
  return new PinocchioSphereKinematicsCppAd(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioSphereKinematicsCppAd::GetPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                             const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                             const std::vector<SphereApproxParam>& sphereApproxParams,
                                                             const ad_vector_t& state) {
  const PinocchioInterfaceCppAd::Model& model = pinocchioInterfaceCppAd.GetModel();
  PinocchioInterfaceCppAd::Data& data = pinocchioInterfaceCppAd.GetData();
  const ad_vector_t q = mapping.GetPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateGlobalPlacements(model, data);

  const pinocchio::GeometryModel& geometryModel = pinocchioSphereInterface_.GetGeometryModel();
  const size_t numPrimitiveShapes = pinocchioSphereInterface_.GetNumPrimitiveShapes();
  const size_array_t geomObjIds = pinocchioSphereInterface_.GetGeomObjIds();
  const size_array_t numSpheres = pinocchioSphereInterface_.GetNumSpheres();
  ad_vector_t sphereCentersInWorldFrame(3 * pinocchioSphereInterface_.GetNumSpheresInTotal());

  size_t count = 0;
  for (size_t i = 0; i < numPrimitiveShapes; i++) {
    const size_t parentJointId = geometryModel.geometryObjects[geomObjIds[i]].parentJoint;
    const ad_vector_t& translation = sphereApproxParams[i].placementTranslation.cast<ad_scalar_t>();
    const Eigen::Quaternion<ad_scalar_t>& quaternion = sphereApproxParams[i].placementOrientation.cast<ad_scalar_t>();
    const std::vector<vector3_t>& sphereCentersToObjectCenter = sphereApproxParams[i].sphereCentersToObjectCenter;

    for (size_t j = 0; j < numSpheres[i]; j++) {
      sphereCentersInWorldFrame.segment<3>(count + 3 * j) = data.oMi[parentJointId].translation();
      sphereCentersInWorldFrame.segment<3>(count + 3 * j).noalias() +=
          data.oMi[parentJointId].rotation() *
          (quaternion._transformVector(sphereCentersToObjectCenter[j].cast<ad_scalar_t>()) + translation);
    }

    count += 3 * numSpheres[i];
  }

  return sphereCentersInWorldFrame;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioSphereKinematicsCppAd::GetPosition(const vector_t& state) const -> std::vector<vector3_t> {
  const vector_t positionValues = positionCppAdInterfacePtr_->GetFunctionValue(state);

  std::vector<vector3_t> positions;
  positions.reserve(linkIds_.size());
  for (int i = 0; i < linkIds_.size(); i++) {
    positions.emplace_back(positionValues.segment<3>(3 * i));
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioSphereKinematicsCppAd::GetPositionLinearApproximation(const vector_t& state) const {
  const vector_t positionValues = positionCppAdInterfacePtr_->GetFunctionValue(state);
  const matrix_t positionJacobian = positionCppAdInterfacePtr_->GetJacobian(state);

  std::vector<VectorFunctionLinearApproximation> positions;
  positions.reserve(linkIds_.size());
  for (int i = 0; i < linkIds_.size(); i++) {
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
auto PinocchioSphereKinematicsCppAd::CreateSphereApproxParams() const -> std::vector<SphereApproxParam> {
  const pinocchio::GeometryModel& geometryModel = pinocchioSphereInterface_.GetGeometryModel();
  const size_t numPrimitiveShapes = pinocchioSphereInterface_.GetNumPrimitiveShapes();
  const size_array_t geomObjIds = pinocchioSphereInterface_.GetGeomObjIds();
  std::vector<SphereApproxParam> sphereApproxParams;

  sphereApproxParams.reserve(numPrimitiveShapes);
  for (size_t i = 0; i < numPrimitiveShapes; ++i) {
    const auto& placement = geometryModel.geometryObjects[geomObjIds[i]].placement;
    sphereApproxParams.emplace_back(placement.translation(), MatrixToQuaternion(placement.rotation()),
                                    pinocchioSphereInterface_.GetSphereCentersToObjectCenter(i));
  }

  return sphereApproxParams;
}

}  // namespace bolza
