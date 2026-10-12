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

#include "bolza/pinocchio/centroidal/centroidal_model_pinocchio_mapping.hpp"

#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "bolza/pinocchio/centroidal/access_helper_functions.hpp"
#include "bolza/pinocchio/centroidal/model_helper_functions.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
CentroidalModelPinocchioMappingTpl<SCALAR>::CentroidalModelPinocchioMappingTpl(CentroidalModelInfoTpl<SCALAR> centroidalModelInfo)
    : pinocchioInterfacePtr_(nullptr), centroidalModelInfo_(std::move(centroidalModelInfo)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
CentroidalModelPinocchioMappingTpl<SCALAR>::CentroidalModelPinocchioMappingTpl(const CentroidalModelPinocchioMappingTpl& rhs)
    : pinocchioInterfacePtr_(nullptr), centroidalModelInfo_(rhs.centroidalModelInfo_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
CentroidalModelPinocchioMappingTpl<SCALAR>* CentroidalModelPinocchioMappingTpl<SCALAR>::Clone() const {
  return new CentroidalModelPinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
void CentroidalModelPinocchioMappingTpl<SCALAR>::SetPinocchioInterface(const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface) {
  pinocchioInterfacePtr_ = &pinocchioInterface;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMappingTpl<SCALAR>::GetPinocchioJointPosition(const vector_t& state) const -> vector_t {
  return centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMappingTpl<SCALAR>::GetPinocchioJointVelocity(const vector_t& state, const vector_t& input) const -> vector_t {
  const auto& model = pinocchioInterfacePtr_->getModel();
  const auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = centroidalModelInfo_;
  assert(info.stateDim == state.rows());
  assert(info.inputDim == input.rows());

  const auto& A = GetCentroidalMomentumMatrix(*pinocchioInterfacePtr_);
  const Eigen::Matrix<SCALAR, 6, 6> Ab = A.template leftCols<6>();
  const auto Ab_inv = ComputeFloatingBaseCentroidalMomentumMatrixInverse(Ab);

  const auto jointVelocities = centroidal_model::GetJointVelocities(input, info).head(info.actuatedDofNum);

  Eigen::Matrix<SCALAR, 6, 1> momentum = info.robotMass * centroidal_model::GetNormalizedMomentum(state, info);
  if (info.centroidalModelType == CentroidalModelType::FullCentroidalDynamics) {
    momentum.noalias() -= A.rightCols(info.actuatedDofNum) * jointVelocities;
  }

  vector_t vPinocchio(info.generalizedCoordinatesNum);
  vPinocchio.template head<6>().noalias() = Ab_inv * momentum;
  vPinocchio.tail(info.actuatedDofNum) = jointVelocities;

  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMappingTpl<SCALAR>::GetOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  const auto& model = pinocchioInterfacePtr_->getModel();
  const auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = centroidalModelInfo_;
  assert(info.stateDim == state.rows());

  // Partial derivatives of joint velocities
  matrix_t jointVelocitiesDerivativeInput = matrix_t::Zero(info.actuatedDofNum, info.inputDim);
  jointVelocitiesDerivativeInput.rightCols(info.actuatedDofNum).setIdentity();

  // Partial derivatives of the floating base variables
  // TODO: move getFloatingBaseCentroidalMomentumMatrixInverse(Ab) to PreComputation
  matrix_t floatingBaseVelocitiesDerivativeState = matrix_t::Zero(6, info.stateDim);
  matrix_t floatingBaseVelocitiesDerivativeInput = matrix_t::Zero(6, info.inputDim);
  const auto& A = GetCentroidalMomentumMatrix(*pinocchioInterfacePtr_);
  const Eigen::Matrix<SCALAR, 6, 6> Ab = A.template leftCols<6>();
  const auto Ab_inv = ComputeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  floatingBaseVelocitiesDerivativeState.leftCols(6) = info.robotMass * Ab_inv;

  using matrix6x_t = Eigen::Matrix<SCALAR, 6, Eigen::Dynamic>;
  matrix6x_t dhdq(6, info.generalizedCoordinatesNum);
  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      pinocchio::translateForceSet(data.dHdq, data.com[0], dhdq.const_cast_derived());
      for (size_t k = 0; k < model.nv; ++k) {
        dhdq.template block<3, 1>(pinocchio::Force::ANGULAR, k) +=
            data.hg.linear().cross(data.dFda.template block<3, 1>(pinocchio::Force::LINEAR, k)) / data.Ig.mass();
      }
      dhdq.middleCols(3, 3) = data.dFdq.middleCols(3, 3);
      const auto Aj = A.rightCols(info.actuatedDofNum);
      floatingBaseVelocitiesDerivativeState.rightCols(info.generalizedCoordinatesNum).noalias() = -Ab_inv * dhdq;
      floatingBaseVelocitiesDerivativeInput.rightCols(info.actuatedDofNum).noalias() = -Ab_inv * Aj;
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      dhdq = data.dFdq;
      floatingBaseVelocitiesDerivativeState.middleCols(6, 6).noalias() = -Ab_inv * dhdq.leftCols(6);
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
    }
  }

  matrix_t dvdx = matrix_t::Zero(info.generalizedCoordinatesNum, info.stateDim);
  dvdx.template topRows<6>() = floatingBaseVelocitiesDerivativeState;
  matrix_t dvdu = matrix_t::Zero(info.generalizedCoordinatesNum, info.inputDim);
  dvdu << floatingBaseVelocitiesDerivativeInput, jointVelocitiesDerivativeInput;
  matrix_t dfdx = matrix_t::Zero(Jq.rows(), centroidalModelInfo_.stateDim);
  dfdx.middleCols(6, info.generalizedCoordinatesNum) = Jq;
  dfdx.noalias() += Jv * dvdx;
  const matrix_t dfdu = Jv * dvdu;
  return {dfdx, dfdu};
}

// explicit template instantiation
template class bolza::CentroidalModelPinocchioMappingTpl<bolza::scalar_t>;
template class bolza::CentroidalModelPinocchioMappingTpl<bolza::ad_scalar_t>;

}  // namespace bolza
