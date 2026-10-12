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

#include "bolza/perceptive/end_effector/end_effector_distance_constraint_cpp_ad.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorDistanceConstraintCppAd::EndEffectorDistanceConstraintCppAd(size_t stateDim, size_t inputDim, Config config,
                                                                       std::unique_ptr<EndEffectorKinematics<ad_scalar_t>> adKinematicsPtr)
    : StateInputConstraint(ConstraintOrder::Linear),
      stateDim_(stateDim),
      inputDim_(inputDim),
      config_(std::move(config)),
      adKinematicsPtr_(std::move(adKinematicsPtr)) {
  auto surrogateKinematicsAD = [this](const ad_vector_t& x, ad_vector_t& y) {
    const size_t numEEs = adKinematicsPtr_->GetIds().size();
    const auto eePositions = adKinematicsPtr_->GetPosition(x);
    y.resize(3 * numEEs);
    for (size_t i = 0; i < numEEs; i++) {
      y.segment<3>(3 * i) = eePositions[i];
    }  // end of i loop
  };

  std::string libName = "ee_kinemtaics";
  for (const auto& id : adKinematicsPtr_->GetIds()) {
    libName += "_" + id;
  }
  kinematicsModelPtr_.reset(new CppAdInterface(surrogateKinematicsAD, stateDim_, libName));
  constexpr auto order = CppAdInterface::ApproximationOrder::First;
  if (config_.generateModel) {
    kinematicsModelPtr_->CreateModels(order, config_.verbose);
  } else {
    kinematicsModelPtr_->LoadModelsIfAvailable(order, config_.verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorDistanceConstraintCppAd::EndEffectorDistanceConstraintCppAd(const EndEffectorDistanceConstraintCppAd& other)
    : StateInputConstraint(other),
      stateDim_(other.stateDim_),
      inputDim_(other.inputDim_),
      config_(other.config_),
      adKinematicsPtr_(other.adKinematicsPtr_->Clone()),
      kinematicsModelPtr_(new CppAdInterface(*other.kinematicsModelPtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorDistanceConstraintCppAd::set(scalar_t clearance, const DistanceTransformInterface& distanceTransform) {
  const auto numEEs = adKinematicsPtr_->GetIds().size();
  clearances_ = vector_t::Ones(numEEs) * clearance;
  distanceTransformPtr_ = &distanceTransform;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorDistanceConstraintCppAd::set(vector_t clearances, const DistanceTransformInterface& distanceTransform) {
  const auto numEEs = adKinematicsPtr_->GetIds().size();
  if (clearances.size() != numEEs) {
    throw std::runtime_error(
        "[EndEffectorDistanceConstraint] The size of the input clearance vector doesn't match the number of end-effectors!");
  }

  clearances_ = clearances;
  distanceTransformPtr_ = &distanceTransform;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorDistanceConstraintCppAd::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                      const PreComputation& preComp) const {
  const auto numEEs = adKinematicsPtr_->GetIds().size();
  const auto eePositions = kinematicsModelPtr_->GetFunctionValue(state);
  assert(eePositions.size() == 3 * numEEs);

  vector_t g(numEEs);
  for (size_t i = 0; i < numEEs; i++) {
    g(i) = config_.weight * (distanceTransformPtr_->GetValue(eePositions.segment<3>(3 * i)) - clearances_[i]);
  }  // end of i loop

  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorDistanceConstraintCppAd::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                             const vector_t& input,
                                                                                             const PreComputation& preComp) const {
  const auto numEEs = adKinematicsPtr_->GetIds().size();
  const auto eePositions = kinematicsModelPtr_->GetFunctionValue(state);
  const auto eeJacobians = kinematicsModelPtr_->GetJacobian(state);
  assert(eePositions.size() == 3 * numEEs);
  assert(eeJacobians.rows() == 3 * numEEs);
  assert(eeJacobians.cols() == stateDim_);

  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(numEEs, stateDim_, inputDim_);
  for (size_t i = 0; i < numEEs; i++) {
    const auto distanceValueGradient = distanceTransformPtr_->GetLinearApproximation(eePositions.segment<3>(3 * i));
    approx.f(i) = config_.weight * (distanceValueGradient.first - clearances_[i]);
    approx.dfdx.row(i).noalias() = config_.weight * (distanceValueGradient.second.transpose() * eeJacobians.middleRows<3>(3 * i));
  }  // end of i loop

  return approx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation EndEffectorDistanceConstraintCppAd::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                                   const vector_t& input,
                                                                                                   const PreComputation& preComp) const {
  const auto numEEs = adKinematicsPtr_->GetIds().size();
  auto linearApprox = GetLinearApproximation(time, state, input, preComp);

  VectorFunctionQuadraticApproximation quadraticApproximation;
  quadraticApproximation.f = std::move(linearApprox.f);
  quadraticApproximation.dfdx = std::move(linearApprox.dfdx);
  quadraticApproximation.dfdu = std::move(linearApprox.dfdu);
  quadraticApproximation.dfdxx.assign(numEEs, matrix_t::Zero(stateDim_, stateDim_));
  quadraticApproximation.dfdux.assign(numEEs, matrix_t::Zero(inputDim_, stateDim_));
  quadraticApproximation.dfduu.assign(numEEs, matrix_t::Zero(inputDim_, inputDim_));

  return quadraticApproximation;
}

}  // namespace bolza
