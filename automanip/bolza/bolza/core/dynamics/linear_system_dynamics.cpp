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

#include <bolza/core/dynamics/linear_system_dynamics.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearSystemDynamics::LinearSystemDynamics(matrix_t A, matrix_t B, matrix_t G /*= matrix_t()*/)
    : A_(std::move(A)), B_(std::move(B)), G_(std::move(G)) {
  if (G_.size() == 0) {
    G_.setIdentity(A_.rows(), A_.rows());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearSystemDynamics* LinearSystemDynamics::Clone() const {
  return new LinearSystemDynamics(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearSystemDynamics::ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) {
  vector_t f = A_ * x;
  f.noalias() += B_ * u;
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearSystemDynamics::ComputeJumpMap(scalar_t t, const vector_t& x, const PreComputation&) {
  return G_ * x;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearSystemDynamics::LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                            const PreComputation&) {
  VectorFunctionLinearApproximation approximation;
  approximation.f = A_ * x;
  approximation.f.noalias() += B_ * u;
  approximation.dfdx = A_;
  approximation.dfdu = B_;
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearSystemDynamics::JumpMapLinearApproximation(scalar_t t, const vector_t& x, const PreComputation&) {
  VectorFunctionLinearApproximation approximation;
  approximation.f = G_ * x;
  approximation.dfdx = G_;
  approximation.dfdu.setZero(A_.rows(), 0);
  return approximation;
}

}  // namespace bolza
