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

#include "examples/legged_robot/constraint/friction_cone_constraint.hpp"

#include <bolza/pinocchio/centroidal/access_helper_functions.hpp>

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::FrictionConeConstraint(const SwitchedModelReferenceManager& referenceManager, Config config,
                                               size_t contactPointIndex, CentroidalModelInfo info)
    : StateInputConstraint(ConstraintOrder::Quadratic),
      referenceManagerPtr_(&referenceManager),
      config_(std::move(config)),
      contactPointIndex_(contactPointIndex),
      info_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrictionConeConstraint::SetSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld) {
  t_R_w.setIdentity();
  throw std::runtime_error("[FrictionConeConstraint] SetSurfaceNormalInWorld() is not implemented!");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FrictionConeConstraint::IsActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FrictionConeConstraint::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                          const PreComputation& preComp) const {
  const auto forcesInWorldFrame = centroidal_model::GetContactForces(input, contactPointIndex_, info_);
  const vector3_t localForce = t_R_w * forcesInWorldFrame;
  return ConeConstraint(localForce);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation FrictionConeConstraint::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input,
                                                                                 const PreComputation& preComp) const {
  const vector3_t forcesInWorldFrame = centroidal_model::GetContactForces(input, contactPointIndex_, info_);
  const vector3_t localForce = t_R_w * forcesInWorldFrame;

  const auto localForceDerivatives = ComputeLocalForceDerivatives(forcesInWorldFrame);
  const auto coneLocalDerivatives = ComputeConeLocalDerivatives(localForce);
  const auto coneDerivatives = ComputeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = ConeConstraint(localForce);
  linearApproximation.dfdx = matrix_t::Zero(1, state.size());
  linearApproximation.dfdu = FrictionConeInputDerivative(input.size(), coneDerivatives);
  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation FrictionConeConstraint::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                       const vector_t& input,
                                                                                       const PreComputation& preComp) const {
  const vector3_t forcesInWorldFrame = centroidal_model::GetContactForces(input, contactPointIndex_, info_);
  const vector3_t localForce = t_R_w * forcesInWorldFrame;

  const auto localForceDerivatives = ComputeLocalForceDerivatives(forcesInWorldFrame);
  const auto coneLocalDerivatives = ComputeConeLocalDerivatives(localForce);
  const auto coneDerivatives = ComputeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  VectorFunctionQuadraticApproximation quadraticApproximation;
  quadraticApproximation.f = ConeConstraint(localForce);
  quadraticApproximation.dfdx = matrix_t::Zero(1, state.size());
  quadraticApproximation.dfdu = FrictionConeInputDerivative(input.size(), coneDerivatives);
  quadraticApproximation.dfdxx.emplace_back(FrictionConeSecondDerivativeState(state.size(), coneDerivatives));
  quadraticApproximation.dfduu.emplace_back(FrictionConeSecondDerivativeInput(input.size(), coneDerivatives));
  quadraticApproximation.dfdux.emplace_back(matrix_t::Zero(input.size(), state.size()));
  return quadraticApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::LocalForceDerivatives FrictionConeConstraint::ComputeLocalForceDerivatives(
    const vector3_t& forcesInWorldFrame) const {
  LocalForceDerivatives localForceDerivatives{};
  localForceDerivatives.dF_du = t_R_w;
  return localForceDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::ConeLocalDerivatives FrictionConeConstraint::ComputeConeLocalDerivatives(const vector3_t& localForces) const {
  const auto F_x_square = localForces.x() * localForces.x();
  const auto F_y_square = localForces.y() * localForces.y();
  const auto F_tangent_square = F_x_square + F_y_square + config_.regularization;
  const auto F_tangent_norm = sqrt(F_tangent_square);
  const auto F_tangent_square_pow32 = F_tangent_norm * F_tangent_square;  // = F_tangent_square ^ (3/2)

  ConeLocalDerivatives coneDerivatives{};
  coneDerivatives.dCone_dF(0) = -localForces.x() / F_tangent_norm;
  coneDerivatives.dCone_dF(1) = -localForces.y() / F_tangent_norm;
  coneDerivatives.dCone_dF(2) = config_.frictionCoefficient;

  coneDerivatives.d2Cone_dF2(0, 0) = -(F_y_square + config_.regularization) / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(0, 1) = localForces.x() * localForces.y() / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(0, 2) = 0.0;
  coneDerivatives.d2Cone_dF2(1, 0) = coneDerivatives.d2Cone_dF2(0, 1);
  coneDerivatives.d2Cone_dF2(1, 1) = -(F_x_square + config_.regularization) / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(1, 2) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 0) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 1) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 2) = 0.0;

  return coneDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FrictionConeConstraint::ConeConstraint(const vector3_t& localForces) const {
  const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + config_.regularization;
  const auto F_tangent_norm = sqrt(F_tangent_square);
  const scalar_t coneConstraint = config_.frictionCoefficient * (localForces.z() + config_.gripperForce) - F_tangent_norm;
  return (vector_t(1) << coneConstraint).finished();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrictionConeConstraint::ConeDerivatives FrictionConeConstraint::ComputeConeConstraintDerivatives(
    const ConeLocalDerivatives& coneLocalDerivatives, const LocalForceDerivatives& localForceDerivatives) const {
  ConeDerivatives coneDerivatives;
  // First order derivatives
  coneDerivatives.dCone_du.noalias() = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_du;

  // Second order derivatives
  coneDerivatives.d2Cone_du2.noalias() =
      localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;

  return coneDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionConeConstraint::FrictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const {
  matrix_t dhdu = matrix_t::Zero(1, inputDim);
  dhdu.block<1, 3>(0, 3 * contactPointIndex_) = coneDerivatives.dCone_du;
  return dhdu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionConeConstraint::FrictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdudu = matrix_t::Zero(inputDim, inputDim);
  ddhdudu.block<3, 3>(3 * contactPointIndex_, 3 * contactPointIndex_) = coneDerivatives.d2Cone_du2;
  ddhdudu.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdudu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FrictionConeConstraint::FrictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdxdx = matrix_t::Zero(stateDim, stateDim);
  ddhdxdx.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdxdx;
}

}  // namespace legged_robot
}  // namespace bolza
