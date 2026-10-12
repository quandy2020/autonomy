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

#include <gtest/gtest.h>

#include "testLoopshapingConfigurations.h"

#include <bolza/core/loopshaping/loopshaping_property_tree.hpp>

using namespace bolza;

TEST(testLoopshapingDefinition, readingAllDefinitions) {
  for (const auto config : configNames) {
    const auto configPath = GetAbsolutePathToConfigurationFile(config);
    auto loopshapingDefinition = loopshaping_property_tree::Load(configPath);
    loopshapingDefinition->Print();
  }

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, stateInputAccessFunctions) {
  for (const auto config : configNames) {
    const auto configPath = GetAbsolutePathToConfigurationFile(config);
    auto loopshapingDefinition = loopshaping_property_tree::Load(configPath);

    const size_t stateDim = loopshapingDefinition->GetInputFilter().GetNumStates();
    const size_t systemStateDim = 1;
    const size_t augmentedstateDim = stateDim + systemStateDim;
    const size_t inputDim = loopshapingDefinition->GetInputFilter().GetNumInputs();

    const vector_t augmentedState = vector_t::Random(augmentedstateDim);
    const vector_t augmentedInput = vector_t::Random(inputDim);

    const vector_t systemState = loopshapingDefinition->GetSystemState(augmentedState);
    ASSERT_TRUE(augmentedState.head(systemStateDim).isApprox(systemState));

    const vector_t filterState = loopshapingDefinition->GetFilterState(augmentedState);
    ASSERT_TRUE(augmentedState.tail(stateDim).isApprox(filterState));

    const vector_t augmentedStateConstructed = loopshapingDefinition->ConcatenateSystemAndFilterState(systemState, filterState);
    ASSERT_TRUE(augmentedState.isApprox(augmentedStateConstructed));

    const vector_t systemInput = loopshapingDefinition->GetSystemInput(augmentedState, augmentedInput);
    const vector_t filteredInput = loopshapingDefinition->GetFilteredInput(augmentedState, augmentedInput);
    const vector_t augmentedInputConstructed = loopshapingDefinition->AugmentedSystemInput(systemInput, filteredInput);
    ASSERT_TRUE(augmentedInput.isApprox(augmentedInputConstructed));

    vector_t equilibriumState;
    vector_t equilibriumInput;
    loopshapingDefinition->GetFilterEquilibrium(systemInput, equilibriumState, equilibriumInput);
  }
}
