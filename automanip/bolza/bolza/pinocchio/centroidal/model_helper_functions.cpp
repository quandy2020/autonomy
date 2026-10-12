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

#include "bolza/pinocchio/centroidal/model_helper_functions.hpp"

#include "bolza/pinocchio/centroidal/access_helper_functions.hpp"

#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void UpdateCentroidalDynamics(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                              const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q) {
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3_t = Eigen::Matrix<SCALAR_T, 3, 3>;
  using matrix6_t = Eigen::Matrix<SCALAR_T, 6, 6>;

  const auto& model = interface.getModel();
  auto& data = interface.getData();

  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      pinocchio::computeCentroidalMap(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      const vector3_t eulerAnglesZyx = q.template segment<3>(3);
      const matrix3_t mappingZyx = GetMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAnglesZyx);
      const matrix3_t rotationBaseToWorld = GetRotationMatrixFromZyxEulerAngles(eulerAnglesZyx);
      const vector3_t comToBasePositionInWorld = rotationBaseToWorld * info.comToBasePositionNominal;
      const matrix3_t skewSymmetricMap = SkewSymmetricMatrix(comToBasePositionInWorld);
      const matrix3_t mat1 = rotationBaseToWorld * info.centroidalInertiaNominal;
      const matrix3_t mat2 = rotationBaseToWorld.transpose() * mappingZyx;
      matrix6_t Ab = matrix6_t::Zero();
      Ab.template topLeftCorner<3, 3>().diagonal().array() = info.robotMass;
      Ab.template topRightCorner<3, 3>().noalias() = info.robotMass * skewSymmetricMap * mappingZyx;
      Ab.template bottomRightCorner<3, 3>().noalias() = mat1 * mat2;
      data.Ag = Eigen::Matrix<SCALAR_T, -1, -1>::Zero(6, info.generalizedCoordinatesNum);
      data.Ag.template leftCols<6>() = Ab;
      data.com[0] = q.template head<3>() - comToBasePositionInWorld;
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
      break;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void UpdateCentroidalDynamicsDerivatives(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                                         const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                         const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v) {
  using matrix6x_t = Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
  const auto& model = interface.getModel();
  auto& data = interface.getData();

  vector_t a = vector_t::Zero(info.generalizedCoordinatesNum);
  matrix6x_t dhdq(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotdq(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotdv(6, info.generalizedCoordinatesNum);
  matrix6x_t dhdotda(6, info.generalizedCoordinatesNum);

  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      pinocchio::computeCentroidalDynamicsDerivatives(model, data, q, v, a, dhdq, dhdotdq, dhdotdv, dhdotda);
      data.Ag = dhdotda;
      // Filling in data.dFdq is a hack since data.dhdq is not available
      data.dFdq.setZero(6, info.generalizedCoordinatesNum);
      data.dFdq.template middleCols<3>(3) = GetCentroidalMomentumZyxGradient(interface, info, q, v);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      // Filling in data.dFdq is a hack since data.dhdq is not available
      data.dFdq.setZero(6, info.generalizedCoordinatesNum);
      data.dFdq.template middleCols<3>(3) = GetCentroidalMomentumZyxGradient(interface, info, q, v);
      pinocchio::computeJointJacobians(model, data, q);
      pinocchio::updateFramePlacements(model, data);
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
const Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>& GetCentroidalMomentumMatrix(const PinocchioInterfaceTpl<SCALAR_T>& interface) {
  return interface.getData().Ag;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> GetPositionComToContactPointInWorldFrame(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                       const CentroidalModelInfoTpl<SCALAR_T>& info, size_t contactIndex) {
  const auto& data = interface.getData();
  return (data.oMf[info.endEffectorFrameIndices[contactIndex]].translation() - data.com[0]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> GetTranslationalJacobianComToContactPointInWorldFrame(
    const PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info, size_t contactIndex) {
  const auto& model = interface.getModel();
  auto data = interface.getData();
  Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic> jacobianWorldToContactPointInWorldFrame;
  jacobianWorldToContactPointInWorldFrame.setZero(6, info.generalizedCoordinatesNum);
  pinocchio::getFrameJacobian(model, data, info.endEffectorFrameIndices[contactIndex], pinocchio::LOCAL_WORLD_ALIGNED,
                              jacobianWorldToContactPointInWorldFrame);
  Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> J_com = GetCentroidalMomentumMatrix(interface).template topRows<3>() / info.robotMass;
  return (jacobianWorldToContactPointInWorldFrame.template topRows<3>() - J_com);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 1> GetNormalizedCentroidalMomentumRate(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                  const CentroidalModelInfoTpl<SCALAR_T>& info,
                                                                  const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input) {
  const Eigen::Matrix<SCALAR_T, 3, 1> gravityVector(SCALAR_T(0.0), SCALAR_T(0.0), SCALAR_T(-9.81));
  Eigen::Matrix<SCALAR_T, 6, 1> centroidalMomentumRate;
  centroidalMomentumRate << info.robotMass * gravityVector, Eigen::Matrix<SCALAR_T, 3, 1>::Zero();

  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto contactForceInWorldFrame = centroidal_model::GetContactForces(input, i, info);
    const auto positionComToContactPointInWorldFrame = GetPositionComToContactPointInWorldFrame(interface, info, i);
    centroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
    centroidalMomentumRate.template tail<3>().noalias() += positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame);
  }  // end of i loop

  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const auto contactForceInWorldFrame = centroidal_model::GetContactForces(input, i, info);
    const auto contactTorqueInWorldFrame = centroidal_model::GetContactTorques(input, i, info);
    const auto positionComToContactPointInWorldFrame = GetPositionComToContactPointInWorldFrame(interface, info, i);
    centroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
    centroidalMomentumRate.template tail<3>().noalias() +=
        positionComToContactPointInWorldFrame.cross(contactForceInWorldFrame) + contactTorqueInWorldFrame;
  }  // end of i loop

  // normalize by the total mass
  centroidalMomentumRate /= info.robotMass;

  return centroidalMomentumRate;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// Explicit template instantiation
template void UpdateCentroidalDynamics<scalar_t>(PinocchioInterface&, const CentroidalModelInfo&, const vector_t&);
template void UpdateCentroidalDynamics<ad_scalar_t>(PinocchioInterfaceCppAd&, const CentroidalModelInfoCppAd&, const ad_vector_t&);

template void UpdateCentroidalDynamicsDerivatives<scalar_t>(PinocchioInterface&, const CentroidalModelInfo& info, const vector_t&,
                                                            const vector_t&);
template void UpdateCentroidalDynamicsDerivatives<ad_scalar_t>(PinocchioInterfaceCppAd&, const CentroidalModelInfoCppAd& info,
                                                               const ad_vector_t&, const ad_vector_t&);

template const Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>& GetCentroidalMomentumMatrix<scalar_t>(const PinocchioInterface&);
template const Eigen::Matrix<ad_scalar_t, 6, Eigen::Dynamic>& GetCentroidalMomentumMatrix<ad_scalar_t>(const PinocchioInterfaceCppAd&);

template Eigen::Matrix<scalar_t, 3, 1> GetPositionComToContactPointInWorldFrame<scalar_t>(const PinocchioInterface&,
                                                                                          const CentroidalModelInfo&, size_t);
template Eigen::Matrix<ad_scalar_t, 3, 1> GetPositionComToContactPointInWorldFrame<ad_scalar_t>(const PinocchioInterfaceCppAd&,
                                                                                                const CentroidalModelInfoCppAd&, size_t);

template Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> GetTranslationalJacobianComToContactPointInWorldFrame<scalar_t>(
    const PinocchioInterface&, const CentroidalModelInfo&, size_t);
template Eigen::Matrix<ad_scalar_t, 3, Eigen::Dynamic> GetTranslationalJacobianComToContactPointInWorldFrame<ad_scalar_t>(
    const PinocchioInterfaceCppAd&, const CentroidalModelInfoCppAd&, size_t);

template Eigen::Matrix<scalar_t, 6, 1> GetNormalizedCentroidalMomentumRate<scalar_t>(const PinocchioInterface&, const CentroidalModelInfo&,
                                                                                     const vector_t&);
template Eigen::Matrix<ad_scalar_t, 6, 1> GetNormalizedCentroidalMomentumRate<ad_scalar_t>(const PinocchioInterfaceCppAd&,
                                                                                           const CentroidalModelInfoCppAd&,
                                                                                           const ad_vector_t&);

}  // namespace bolza
