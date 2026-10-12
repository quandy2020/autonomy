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

#include <bolza/core/constraint/linear_state_constraint.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearStateConstraint::LinearStateConstraint(vector_t h, matrix_t F)
    : StateConstraint(ConstraintOrder::Linear), h_(std::move(h)), F_(std::move(F)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearStateConstraint* LinearStateConstraint::Clone() const {
  return new LinearStateConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t LinearStateConstraint::GetNumConstraints(scalar_t time) const {
  return h_.rows();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearStateConstraint::GetValue(scalar_t t, const vector_t& x, const PreComputation&) const {
  vector_t g = h_;
  g.noalias() += F_ * x;
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearStateConstraint::GetLinearApproximation(scalar_t t, const vector_t& x,
                                                                                const PreComputation&) const {
  VectorFunctionLinearApproximation g;
  g.f = h_;
  g.f.noalias() += F_ * x;
  g.dfdx = F_;
  return g;
}

}  // namespace bolza
