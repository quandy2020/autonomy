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

#include <bolza/core/constraint/state_constraint_collection.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateConstraintCollection::StateConstraintCollection(const StateConstraintCollection& other) : Collection<StateConstraint>(other) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateConstraintCollection* StateConstraintCollection::Clone() const {
  return new StateConstraintCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateConstraintCollection::GetNumConstraints(scalar_t time) const {
  size_t numConstraints = 0;
  for (const auto& constraintTerm : this->terms_) {
    if (constraintTerm->isActive(time)) {
      numConstraints += constraintTerm->getNumConstraints(time);
    }
  }
  return numConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_array_t StateConstraintCollection::GetTermsSize(scalar_t time) const {
  size_array_t termsSize(this->terms_.size(), 0);
  for (size_t i = 0; i < this->terms_.size(); ++i) {
    if (this->terms_[i]->isActive(time)) {
      termsSize[i] = this->terms_[i]->getNumConstraints(time);
    }
  }
  return termsSize;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t StateConstraintCollection::GetValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const {
  vector_array_t constraintValues(this->terms_.size());
  for (size_t i = 0; i < this->terms_.size(); ++i) {
    if (this->terms_[i]->isActive(time)) {
      constraintValues[i] = this->terms_[i]->getValue(time, state, preComp);
    }
  }  // end of i loop
  return constraintValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation StateConstraintCollection::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                    const PreComputation& preComp) const {
  VectorFunctionLinearApproximation LinearApproximation(GetNumConstraints(time), state.rows());

  // append linearApproximation of each constraintTerm
  size_t i = 0;
  for (const auto& constraintTerm : this->terms_) {
    if (constraintTerm->isActive(time)) {
      const auto constraintTermApproximation = constraintTerm->getLinearApproximation(time, state, preComp);
      const size_t nc = constraintTermApproximation.f.rows();
      linearApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      linearApproximation.dfdx.middleRows(i, nc) = constraintTermApproximation.dfdx;
      i += nc;
    }
  }

  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation StateConstraintCollection::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                          const PreComputation& preComp) const {
  const auto numConstraints = GetNumConstraints(time);

  VectorFunctionQuadraticApproximation quadraticApproximation;
  quadraticApproximation.f.resize(numConstraints);
  quadraticApproximation.dfdx.resize(numConstraints, state.rows());
  quadraticApproximation.dfdxx.reserve(numConstraints);  // Use reserve instead of resize to avoid unnecessary allocations.

  // append quadraticApproximation of each constraintTerm
  size_t i = 0;
  for (const auto& constraintTerm : this->terms_) {
    if (constraintTerm->isActive(time)) {
      auto constraintTermApproximation = constraintTerm->getQuadraticApproximation(time, state, preComp);
      const size_t nc = constraintTermApproximation.f.rows();
      quadraticApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      quadraticApproximation.dfdx.middleRows(i, nc) = constraintTermApproximation.dfdx;
      AppendVectorToVectorByMoving(quadraticApproximation.dfdxx, std::move(constraintTermApproximation.dfdxx));
      i += nc;
    }
  }

  return quadraticApproximation;
}

}  // namespace bolza
