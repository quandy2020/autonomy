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

#include <bolza/core/constraint/state_input_constraint_collection.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputConstraintCollection::StateInputConstraintCollection(const StateInputConstraintCollection& other)
    : Collection<StateInputConstraint>(other) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputConstraintCollection* StateInputConstraintCollection::Clone() const {
  return new StateInputConstraintCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateInputConstraintCollection::GetNumConstraints(scalar_t time) const {
  size_t numConstraints = 0;
  for (const auto& constraintTerm : this->terms_) {
    if (constraintTerm->IsActive(time)) {
      numConstraints += constraintTerm->GetNumConstraints(time);
    }
  }
  return numConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_array_t StateInputConstraintCollection::GetTermsSize(scalar_t time) const {
  size_array_t termsSize(this->terms_.size(), 0);
  for (size_t i = 0; i < this->terms_.size(); ++i) {
    if (this->terms_[i]->IsActive(time)) {
      termsSize[i] = this->terms_[i]->GetNumConstraints(time);
    }
  }
  return termsSize;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t StateInputConstraintCollection::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                        const PreComputation& preComp) const {
  vector_array_t constraintValues(this->terms_.size());
  for (size_t i = 0; i < this->terms_.size(); ++i) {
    if (this->terms_[i]->IsActive(time)) {
      constraintValues[i] = this->terms_[i]->GetValue(time, state, input, preComp);
    }
  }  // end of i loop
  return constraintValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation StateInputConstraintCollection::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                         const vector_t& input,
                                                                                         const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation(GetNumConstraints(time), state.rows(), input.rows());

  // append linearApproximation of each constraintTerm
  size_t i = 0;
  for (const auto& constraintTerm : this->terms_) {
    if (constraintTerm->IsActive(time)) {
      const auto constraintTermApproximation = constraintTerm->GetLinearApproximation(time, state, input, preComp);
      const size_t nc = constraintTermApproximation.f.rows();
      linearApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      linearApproximation.dfdx.middleRows(i, nc) = constraintTermApproximation.dfdx;
      linearApproximation.dfdu.middleRows(i, nc) = constraintTermApproximation.dfdu;
      i += nc;
    }
  }

  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation StateInputConstraintCollection::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                               const vector_t& input,
                                                                                               const PreComputation& preComp) const {
  const auto numConstraints = GetNumConstraints(time);

  VectorFunctionQuadraticApproximation quadraticApproximation;
  quadraticApproximation.f.resize(numConstraints);
  quadraticApproximation.dfdx.resize(numConstraints, state.rows());
  quadraticApproximation.dfdu.resize(numConstraints, input.rows());
  quadraticApproximation.dfdxx.reserve(numConstraints);  // Use reserve instead of resize to avoid unnecessary allocations.
  quadraticApproximation.dfdux.reserve(numConstraints);
  quadraticApproximation.dfduu.reserve(numConstraints);

  // append quadraticApproximation of each constraintTerm
  size_t i = 0;
  for (const auto& constraintTerm : this->terms_) {
    if (constraintTerm->IsActive(time)) {
      auto constraintTermApproximation = constraintTerm->GetQuadraticApproximation(time, state, input, preComp);
      const size_t nc = constraintTermApproximation.f.rows();
      quadraticApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      quadraticApproximation.dfdx.middleRows(i, nc) = constraintTermApproximation.dfdx;
      quadraticApproximation.dfdu.middleRows(i, nc) = constraintTermApproximation.dfdu;
      AppendVectorToVectorByMoving(quadraticApproximation.dfdxx, std::move(constraintTermApproximation.dfdxx));
      AppendVectorToVectorByMoving(quadraticApproximation.dfdux, std::move(constraintTermApproximation.dfdux));
      AppendVectorToVectorByMoving(quadraticApproximation.dfduu, std::move(constraintTermApproximation.dfduu));
      i += nc;
    }
  }

  return quadraticApproximation;
}

}  // namespace bolza
