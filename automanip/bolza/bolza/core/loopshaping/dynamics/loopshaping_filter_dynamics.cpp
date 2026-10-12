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

#include <bolza/core/integration/ode_func.hpp>
#include <bolza/core/loopshaping/dynamics/loopshaping_filter_dynamics.hpp>

namespace bolza {

void LoopshapingFilterDynamics::Integrate(scalar_t dt, const vector_t& input) {
  // Set up ODE with ZOH input
  OdeFunc ode_fun(std::bind(&LoopshapingFilterDynamics::ComputeFlowMap, this, std::placeholders::_1, std::placeholders::_2, input));

  vector_array_t stateTrajectory;
  Observer Observer(&stateTrajectory);
  integrator_->integrateAdaptive(ode_fun, observer, filterState_, 0.0, dt, dt);

  filterState_ = stateTrajectory.back();
}

vector_t LoopshapingFilterDynamics::ComputeFlowMap(scalar_t time, const vector_t& filterState, const vector_t& input) const {
  return loopshapingDefinition_->filterFlowMap(filterState, input);
}

}  // namespace bolza
