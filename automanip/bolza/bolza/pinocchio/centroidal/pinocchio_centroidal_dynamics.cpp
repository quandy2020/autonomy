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

#include "bolza/pinocchio/centroidal/pinocchio_centroidal_dynamics.hpp"

#include <bolza/tools/common/skew_symmetric_matrix.hpp>

#include "bolza/pinocchio/centroidal/access_helper_functions.hpp"
#include "bolza/pinocchio/centroidal/model_helper_functions.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioCentroidalDynamics::PinocchioCentroidalDynamics(CentroidalModelInfo info)
    : pinocchioInterfacePtr_(nullptr), mapping_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioCentroidalDynamics::PinocchioCentroidalDynamics(const PinocchioCentroidalDynamics& rhs)
    : pinocchioInterfacePtr_(nullptr), mapping_(rhs.mapping_.getCentroidalModelInfo()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PinocchioCentroidalDynamics::GetValue(scalar_t time, const vector_t& state, const vector_t& input) {
  const auto& interface = *pinocchioInterfacePtr_;
  const auto& info = mapping_.getCentroidalModelInfo();
  assert(info.stateDim == state.rows());

  vector_t f(info.stateDim);
  f << GetNormalizedCentroidalMomentumRate(interface, info, input), mapping_.getPinocchioJointVelocity(state, input);

  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation PinocchioCentroidalDynamics::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                      const vector_t& input) {
  const auto& info = mapping_.getCentroidalModelInfo();
  assert(info.stateDim == state.rows());
  assert(info.inputDim == input.rows());

  auto dynamics = bolza::VectorFunctionLinearApproximation::Zero(info.stateDim, info.stateDim, info.inputDim);
  dynamics.f = GetValue(time, state, input);

  // Partial derivatives of the normalized momentum rates
  ComputeNormalizedCentroidalMomentumRateGradients(state, input);

  matrix_t dfdq = matrix_t::Zero(info.stateDim, info.generalizedCoordinatesNum);
  matrix_t dfdv = matrix_t::Zero(info.stateDim, info.generalizedCoordinatesNum);
  dfdq.topRows<6>() << normalizedLinearMomentumRateDerivativeQ_, normalizedAngularMomentumRateDerivativeQ_;
  dfdv.bottomRows(info.generalizedCoordinatesNum).setIdentity();

  std::tie(dynamics.dfdx, dynamics.dfdu) = mapping_.getOcs2Jacobian(state, dfdq, dfdv);

  // Add partial derivative of f with respect to u since part of f depends explicitly on the inputs (contact forces + torques)
  dynamics.dfdu.topRows<3>() += normalizedLinearMomentumRateDerivativeInput_;
  dynamics.dfdu.middleRows(3, 3) += normalizedAngularMomentumRateDerivativeInput_;

  return dynamics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PinocchioCentroidalDynamics::ComputeNormalizedCentroidalMomentumRateGradients(const vector_t& state, const vector_t& input) {
  const auto& interface = *pinocchioInterfacePtr_;
  const auto& info = mapping_.getCentroidalModelInfo();
  assert(info.stateDim == state.rows());
  assert(info.inputDim == input.rows());

  // compute partial derivatives of the center of robotMass acceleration and normalized angular momentum
  normalizedLinearMomentumRateDerivativeQ_.setZero(3, info.generalizedCoordinatesNum);
  normalizedAngularMomentumRateDerivativeQ_.setZero(3, info.generalizedCoordinatesNum);
  normalizedLinearMomentumRateDerivativeInput_.setZero(3, info.inputDim);
  normalizedAngularMomentumRateDerivativeInput_.setZero(3, info.inputDim);
  Matrix3 f_hat, p_hat;

  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const Vector3 contactForceInWorldFrame = centroidal_model::GetContactForces(input, i, info);
    f_hat = SkewSymmetricMatrix(contactForceInWorldFrame) / info.robotMass;
    const auto J = GetTranslationalJacobianComToContactPointInWorldFrame(interface, info, i);
    normalizedAngularMomentumRateDerivativeQ_.noalias() -= f_hat * J;
    normalizedLinearMomentumRateDerivativeInput_.block<3, 3>(0, 3 * i).diagonal().array() = 1.0 / info.robotMass;
    p_hat = SkewSymmetricMatrix(GetPositionComToContactPointInWorldFrame(interface, info, i)) / info.robotMass;
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * i) = p_hat;
  }

  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const size_t inputIdx = 3 * info.numThreeDofContacts + 6 * (i - info.numThreeDofContacts);
    const Vector3 contactForceInWorldFrame = centroidal_model::GetContactForces(input, i, info);
    f_hat = SkewSymmetricMatrix(contactForceInWorldFrame) / info.robotMass;
    const auto J = GetTranslationalJacobianComToContactPointInWorldFrame(interface, info, i);
    normalizedAngularMomentumRateDerivativeQ_.noalias() -= f_hat * J;
    normalizedLinearMomentumRateDerivativeInput_.block<3, 3>(0, inputIdx).diagonal().array() = 1.0 / info.robotMass;
    p_hat = SkewSymmetricMatrix(GetPositionComToContactPointInWorldFrame(interface, info, i)) / info.robotMass;
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * inputIdx) = p_hat;
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * inputIdx + 3).diagonal().array() = 1.0 / info.robotMass;
  }
}

}  // namespace bolza
