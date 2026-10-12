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

#include "testLoopshapingConfigurations.h"

namespace bolza {

LoopshapingTestConfiguration::LoopshapingTestConfiguration(const std::string& configName) {
  // Load loopshaping definition
  const std::string settingsFile = GetAbsolutePathToConfigurationFile(configName);
  loopshapingDefinition_ = loopshaping_property_tree::Load(settingsFile);

  systemStateDim_ = 1;  // Doesn't matter, just there to create a different size augmented state.
  filterStateDim_ = loopshapingDefinition_->GetInputFilter().GetNumStates();
  inputDim_ = loopshapingDefinition_->GetInputFilter().GetNumInputs();

  // Set up state and input
  t = 0.5;
  const scalar_t eps = 1e-2;
  GetRandomStateInput(systemStateDim_, filterStateDim_, inputDim_, x_sys_, u_sys_, x_filter_, u_filter_, x_, u_);
  GetRandomStateInput(systemStateDim_, filterStateDim_, inputDim_, x_sys_disturbance_, u_sys_disturbance_, x_filter_disturbance_,
                      u_filter_disturbance_, x_disturbance_, u_disturbance_, eps);

  preComp_sys_.reset(new PreComputation);
  preComp_.reset(new LoopshapingPreComputation(*preComp_sys_, loopshapingDefinition_));
};

void LoopshapingTestConfiguration::GetRandomStateInput(size_t systemStateDim, size_t filterStateDim, size_t inputDim, vector_t& x_sys,
                                                       vector_t& u_sys, vector_t& x_filter, vector_t& u_filter, vector_t& x, vector_t& u,
                                                       scalar_t range) {
  // Set random state
  x.setRandom(systemStateDim + filterStateDim);
  u.setRandom(inputDim);

  // Scale the randomness
  x *= range;
  u *= range;

  // Retreive system and filter state
  x_sys = loopshapingDefinition_->GetSystemState(x);
  u_sys = loopshapingDefinition_->GetSystemInput(x, u);
  x_filter = loopshapingDefinition_->GetFilterState(x);
  u_filter = loopshapingDefinition_->GetFilteredInput(x, u);
}

}  // namespace bolza
