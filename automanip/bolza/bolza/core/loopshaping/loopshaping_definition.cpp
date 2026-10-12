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

#include "bolza/core/loopshaping/loopshaping_definition.hpp"

#include <iostream>

namespace bolza {

LoopshapingDefinition::LoopshapingDefinition(LoopshapingType loopshapingType, Filter filter, matrix_t costMatrix)
    : loopshapingType_(loopshapingType), filter_(std::move(filter)), R_(std::move(costMatrix)) {
  if (filter_.GetNumStates() == 0) {
    throw std::runtime_error(
        "[LoopshapingDefinition] The definition has zero extra states. This would be equivalent to a constant scaling. Using loopshaping "
        "does not make sense");
  }

  // Detect diagonal formulation if all involved matrices are diagonal
  diagonal_ = filter_.GetA().isDiagonal() && filter_.GetB().isDiagonal() && filter_.GetC().isDiagonal() && filter_.GetD().isDiagonal();

  if (R_.size() == 0) {  // No cost provided
    R_.setIdentity(filter_.GetNumInputs(), filter_.GetNumInputs());
  }
}

void LoopshapingDefinition::Print() const {
  std::cerr << "[LoopshapingDefinition] \n";
  filter_.Print();
  std::cerr << "R on filtered inputs: \n" << R_ << std::endl;
}

vector_t LoopshapingDefinition::GetSystemInput(const vector_t& state, const vector_t& input) const {
  vector_t systemInput;
  GetSystemInput(state, input, systemInput);
  return systemInput;
}

void LoopshapingDefinition::GetSystemInput(const vector_t& state, const vector_t& input, vector_t& systemInput) const {
  switch (loopshapingType_) {
    case LoopshapingType::outputpattern: {
      systemInput = input;
      break;
    }
    case LoopshapingType::eliminatepattern: {
      if (diagonal_) {
        systemInput = filter_.GetCdiag().diagonal().cwiseProduct(state.tail(filter_.GetNumStates())) +
                      filter_.GetDdiag().diagonal().cwiseProduct(input);
      } else {
        // u = C*x + D*v. Use noalias to prevent temporaries.
        systemInput.noalias() = filter_.GetC() * state.tail(filter_.GetNumStates());
        systemInput.noalias() += filter_.GetD() * input;
      }
      break;
    }
    default:
      throw std::runtime_error("[LoopshapingDefinition::GetSystemInput] invalid loopshaping type");
  }
}

vector_t LoopshapingDefinition::GetFilteredInput(const vector_t& state, const vector_t& input) const {
  vector_t filteredInput;
  GetFilteredInput(state, input, filteredInput);
  return filteredInput;
}

void LoopshapingDefinition::GetFilteredInput(const vector_t& state, const vector_t& input, vector_t& filteredInput) const {
  switch (loopshapingType_) {
    case LoopshapingType::outputpattern: {
      if (diagonal_) {
        filteredInput = filter_.GetCdiag().diagonal().cwiseProduct(state.tail(filter_.GetNumStates())) +
                        filter_.GetDdiag().diagonal().cwiseProduct(input);
      } else {
        filteredInput.noalias() = filter_.GetC() * state.tail(filter_.GetNumStates());
        filteredInput.noalias() += filter_.GetD() * input;
      }
      break;
    }
    case LoopshapingType::eliminatepattern: {
      filteredInput = input;
      break;
    }
    default:
      throw std::runtime_error("[LoopshapingDefinition::GetFilteredInput] invalid loopshaping type");
  }
}

vector_t LoopshapingDefinition::FilterFlowMap(const vector_t& filterState, const vector_t& input) const {
  // Same equation for both loopshaping types
  if (diagonal_) {
    return filter_.GetAdiag().diagonal().cwiseProduct(filterState) + filter_.GetBdiag().diagonal().cwiseProduct(input);
  } else {
    vector_t filterStateDerivative = filter_.GetA() * filterState;
    filterStateDerivative.noalias() += filter_.GetB() * input;
    return filterStateDerivative;
  }
}

vector_t LoopshapingDefinition::ConcatenateSystemAndFilterState(const vector_t& systemState, const vector_t& filterState) const {
  vector_t state(systemState.rows() + filter_.GetNumStates());
  state << systemState, filterState;
  return state;
}

vector_t LoopshapingDefinition::AugmentedSystemInput(const vector_t& systemInput, const vector_t& filterInput) const {
  switch (loopshapingType_) {
    case LoopshapingType::outputpattern:
      return systemInput;
    case LoopshapingType::eliminatepattern:
      return filterInput;
    default:
      throw std::runtime_error("[LoopshapingDefinition::AugmentedSystemInput] invalid loopshaping type");
  }
}

void LoopshapingDefinition::GetFilterEquilibrium(const vector_t& systemInput, vector_t& filterState, vector_t& filterInput) const {
  switch (loopshapingType_) {
    case LoopshapingType::outputpattern:
      filter_.FindEquilibriumForInput(systemInput, filterState, filterInput);
      break;
    case LoopshapingType::eliminatepattern:
      filter_.FindEquilibriumForOutput(systemInput, filterState, filterInput);
      break;
    default:
      throw std::runtime_error("[LoopshapingDefinition::GetFilterEquilibrium] invalid loopshaping type");
  }
}

void LoopshapingDefinition::GetFilterEquilibriumGivenState(const vector_t& systemInput, const vector_t& filterState,
                                                           vector_t& filterInput) const {
  switch (loopshapingType_) {
    case LoopshapingType::outputpattern:
      filterInput = GetFilteredInput(systemInput, filterState);
      break;
    case LoopshapingType::eliminatepattern:
      filter_.FindEquilibriumForOutputGivenState(systemInput, filterState, filterInput);
      break;
    default:
      throw std::runtime_error("[LoopshapingDefinition::GetFilterEquilibriumGivenState] invalid loopshaping type");
  }
}

}  // namespace bolza