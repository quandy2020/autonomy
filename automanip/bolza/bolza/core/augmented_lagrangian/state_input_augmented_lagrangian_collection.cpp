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

#include "bolza/core/augmented_lagrangian/state_input_augmented_lagrangian_collection.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputAugmentedLagrangianCollection* StateInputAugmentedLagrangianCollection::Clone() const {
  return new StateInputAugmentedLagrangianCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateInputAugmentedLagrangianCollection::GetNumberOfActiveConstraints(scalar_t time) const {
  size_t numConstraints = 0;
  for (const auto& term : terms_) {
    if (term->IsActive(time)) {
      numConstraints += term->GetNumConstraints(time);
    }
  }
  return numConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<LagrangianMetrics> StateInputAugmentedLagrangianCollection::GetValue(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input,
                                                                                 const std::vector<Multiplier>& termsMultiplier,
                                                                                 const PreComputation& preComp) const {
  std::vector<LagrangianMetrics> termsConstraintPenalty;
  termsConstraintPenalty.reserve(terms_.size());
  for (size_t i = 0; i < terms_.size(); i++) {
    if (terms_[i]->IsActive(time)) {
      termsConstraintPenalty.emplace_back(terms_[i]->GetValue(time, state, input, termsMultiplier[i], preComp));
    } else {
      termsConstraintPenalty.emplace_back(0.0, vector_t());
    }
  }
  return termsConstraintPenalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputAugmentedLagrangianCollection::GetQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const std::vector<Multiplier>& termsMultiplier,
    const PreComputation& preComp) const {
  const auto firstActiveItr =
      std::find_if(terms_.begin(), terms_.end(),
                   [time](const std::unique_ptr<StateInputAugmentedLagrangianInterface>& costTerm) { return costTerm->IsActive(time); });

  // no active terms (or terms is empty).
  if (firstActiveItr == terms_.end()) {
    return ScalarFunctionQuadraticApproximation::Zero(state.size(), input.size());
  }

  // initialize with first active term
  const size_t firstActiveInd = std::distance(terms_.begin(), firstActiveItr);
  auto penalty = (*firstActiveItr)->GetQuadraticApproximation(time, state, input, termsMultiplier[firstActiveInd], preComp);

  // accumulate terms
  for (size_t i = firstActiveInd + 1; i < terms_.size(); i++) {
    if (terms_[i]->IsActive(time)) {
      penalty += terms_[i]->GetQuadraticApproximation(time, state, input, termsMultiplier[i], preComp);
    }
  }

  return penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputAugmentedLagrangianCollection::UpdateLagrangian(scalar_t time, const vector_t& state, const vector_t& input,
                                                               std::vector<LagrangianMetrics>& termsMetrics,
                                                               std::vector<Multiplier>& termsMultiplier) const {
  assert(termsMetrics.size() == termsMultiplier.size());

  for (size_t i = 0; i < terms_.size(); i++) {
    if (terms_[i]->IsActive(time)) {
      Multiplier updatedLagrangian;
      std::tie(updatedLagrangian, termsMetrics[i].penalty) =
          terms_[i]->UpdateLagrangian(time, state, input, termsMetrics[i].constraint, termsMultiplier[i]);
      termsMultiplier[i] = std::move(updatedLagrangian);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputAugmentedLagrangianCollection::InitializeLagrangian(scalar_t time, std::vector<Multiplier>& termsMultiplier) const {
  termsMultiplier.clear();
  termsMultiplier.reserve(terms_.size());
  for (const auto& term : terms_) {
    if (term->IsActive(time)) {
      termsMultiplier.emplace_back(term->InitializeLagrangian(time));
    } else {
      termsMultiplier.emplace_back(0.0, vector_t());
    }
  }
}

}  // namespace bolza
