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

#include <bolza/core/constraint/linear_state_input_constraint.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearStateInputConstraint::LinearStateInputConstraint(vector_t e, matrix_t C, matrix_t D)
    : StateInputConstraint(ConstraintOrder::Linear), e_(std::move(e)), C_(std::move(C)), D_(std::move(D)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearStateInputConstraint* LinearStateInputConstraint::Clone() const {
  return new LinearStateInputConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t LinearStateInputConstraint::GetNumConstraints(scalar_t time) const {
  return e_.rows();
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearStateInputConstraint::GetValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const {
  vector_t g = e_;
  g.noalias() += C_ * x;
  g.noalias() += D_ * u;
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearStateInputConstraint::GetLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                     const PreComputation&) const {
  VectorFunctionLinearApproximation g;
  g.f = e_;
  g.f.noalias() += C_ * x;
  g.f.noalias() += D_ * u;
  g.dfdx = C_;
  g.dfdu = D_;
  return g;
}

}  // namespace bolza
