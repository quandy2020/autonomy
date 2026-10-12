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

#include "bolza/core/augmented_lagrangian/state_augmented_lagrangian_collection.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateAugmentedLagrangianCollection* StateAugmentedLagrangianCollection::Clone() const {
  return new StateAugmentedLagrangianCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateAugmentedLagrangianCollection::GetNumberOfActiveConstraints(scalar_t time) const {
  size_t numConstraints = 0;
  for (const auto& term : terms_) {
    if (term->isActive(time)) {
      numConstraints += term->getNumConstraints(time);
    }
  }
  return numConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<LagrangianMetrics> StateAugmentedLagrangianCollection::GetValue(scalar_t time, const vector_t& state,
                                                                            const std::vector<Multiplier>& termsMultiplier,
                                                                            const PreComputation& preComp) const {
  std::vector<LagrangianMetrics> termsConstraintPenalty;
  termsConstraintPenalty.reserve(terms_.size());
  for (size_t i = 0; i < terms_.size(); i++) {
    if (terms_[i]->isActive(time)) {
      termsConstraintPenalty.emplace_back(terms_[i]->getValue(time, state, termsMultiplier[i], preComp));
    } else {
      termsConstraintPenalty.emplace_back(0.0, vector_t());
    }
  }
  return termsConstraintPenalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateAugmentedLagrangianCollection::GetQuadraticApproximation(
    scalar_t time, const vector_t& state, const std::vector<Multiplier>& termsMultiplier, const PreComputation& preComp) const {
  const auto firstActiveItr =
      std::find_if(terms_.begin(), terms_.end(),
                   [time](const std::unique_ptr<StateAugmentedLagrangianInterface>& costTerm) { return costTerm->isActive(time); });

  // no active terms (or terms is empty).
  if (firstActiveItr == terms_.end()) {
    return ScalarFunctionQuadraticApproximation::Zero(state.size());
  }

  // initialize with first active term
  const size_t firstActiveInd = std::Distance(terms_.begin(), firstActiveItr);
  auto penalty = (*firstActiveItr)->getQuadraticApproximation(time, state, termsMultiplier[firstActiveInd], preComp);

  // accumulate terms
  for (size_t i = firstActiveInd + 1; i < terms_.size(); i++) {
    if (terms_[i]->isActive(time)) {
      const auto termPenalty = terms_[i]->getQuadraticApproximation(time, state, termsMultiplier[i], preComp);
      penalty.f += termPenalty.f;
      penalty.dfdx += termPenalty.dfdx;
      penalty.dfdxx += termPenalty.dfdxx;
    }
  }

  // make sure that input derivatives are empty
  penalty.dfdu = vector_t();
  penalty.dfduu = matrix_t();
  penalty.dfdux = matrix_t();

  return penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateAugmentedLagrangianCollection::UpdateLagrangian(scalar_t time, const vector_t& state,
                                                          std::vector<LagrangianMetrics>& termsMetrics,
                                                          std::vector<Multiplier>& termsMultiplier) const {
  assert(termsMetrics.size() == termsMultiplier.size());

  for (size_t i = 0; i < terms_.size(); i++) {
    if (terms_[i]->isActive(time)) {
      Multiplier updatedLagrangian;
      std::tie(updatedLagrangian, termsMetrics[i].penalty) =
          terms_[i]->updateLagrangian(time, state, termsMetrics[i].constraint, termsMultiplier[i]);
      termsMultiplier[i] = std::move(updatedLagrangian);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateAugmentedLagrangianCollection::InitializeLagrangian(scalar_t time, std::vector<Multiplier>& termsMultiplier) const {
  termsMultiplier.clear();
  termsMultiplier.reserve(terms_.size());
  for (const auto& term : terms_) {
    if (term->isActive(time)) {
      termsMultiplier.emplace_back(term->initializeLagrangian(time));
    } else {
      termsMultiplier.emplace_back(0.0, vector_t());
    }
  }
}

}  // namespace bolza
