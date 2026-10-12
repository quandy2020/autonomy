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

#include <bolza/core/cost/state_input_cost_collection.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputCostCollection::StateInputCostCollection(const StateInputCostCollection& other) : Collection<StateInputCost>(other) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputCostCollection* StateInputCostCollection::Clone() const {
  return new StateInputCostCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputCostCollection::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                            const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
  scalar_t cost = 0.0;

  // accumulate cost terms
  for (const auto& costTerm : this->terms_) {
    if (costTerm->IsActive(time)) {
      cost += costTerm->GetValue(time, state, input, targetTrajectories, preComp);
    }
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputCostCollection::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                         const vector_t& input,
                                                                                         const TargetTrajectories& targetTrajectories,
                                                                                         const PreComputation& preComp) const {
  const auto firstActive = std::find_if(terms_.begin(), terms_.end(),
                                        [time](const std::unique_ptr<StateInputCost>& costTerm) { return costTerm->IsActive(time); });

  // No active terms (or terms is empty).
  if (firstActive == terms_.end()) {
    return ScalarFunctionQuadraticApproximation::Zero(state.rows(), input.rows());
  }

  // Initialize with first active term, accumulate potentially other active terms.
  auto cost = (*firstActive)->GetQuadraticApproximation(time, state, input, targetTrajectories, preComp);
  std::for_each(std::next(firstActive), terms_.end(), [&](const std::unique_ptr<StateInputCost>& costTerm) {
    if (costTerm->IsActive(time)) {
      cost += costTerm->GetQuadraticApproximation(time, state, input, targetTrajectories, preComp);
    }
  });

  return cost;
}

}  // namespace bolza
