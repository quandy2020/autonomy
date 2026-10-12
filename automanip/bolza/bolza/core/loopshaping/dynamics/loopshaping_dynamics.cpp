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

#include <bolza/core/loopshaping/dynamics/loopshaping_dynamics.hpp>
#include <bolza/core/loopshaping/dynamics/loopshaping_dynamics_eliminate_pattern.hpp>
#include <bolza/core/loopshaping/dynamics/loopshaping_dynamics_output_pattern.hpp>

namespace bolza {

vector_t LoopshapingDynamics::ComputeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) {
  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();

  const vector_t dynamics_system = systemDynamics_->computeFlowMap(time, x_system, u_system, preComp_system);
  const vector_t dynamics_filter = FilterFlowmap(x_filter, u_filter, u_system);

  return loopshapingDefinition_->concatenateSystemAndFilterState(dynamics_system, dynamics_filter);
}

vector_t LoopshapingDynamics::ComputeJumpMap(scalar_t time, const vector_t& state, const PreComputation& preComp) {
  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto& x_system = preCompLS.getSystemState();

  const vector_t jumpMap_system = systemDynamics_->computeJumpMap(time, x_system, preComp_system);

  // Filter doesn't Jump
  const auto& jumMap_filter = preCompLS.getFilterState();

  return loopshapingDefinition_->concatenateSystemAndFilterState(jumpMap_system, jumMap_filter);
}

vector_t LoopshapingDynamics::ComputeGuardSurfaces(scalar_t time, const vector_t& state) {
  const vector_t x_system = loopshapingDefinition_->getSystemState(state);
  return systemDynamics_->computeGuardSurfaces(time, x_system);
}

VectorFunctionLinearApproximation LoopshapingDynamics::JumpMapLinearApproximation(scalar_t t, const vector_t& x,
                                                                                  const PreComputation& preComp) {
  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);

  // System jump
  const auto& x_system = preCompLS.getSystemState();
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto jumpMap_system = systemDynamics_->jumpMapLinearApproximation(t, x_system, preComp_system);

  // Filter doesn't Jump
  const auto& jumMap_filter = preCompLS.getFilterState();

  VectorFunctionLinearApproximation jumpMap;
  jumpMap.f = loopshapingDefinition_->concatenateSystemAndFilterState(jumpMap_system.f, jumMap_filter);

  const auto postJumpStateDim = jumpMap.f.rows();
  const auto postJumpSysStateDim = jumpMap_system.f.rows();
  const auto postJumpFiltStateDim = jumMap_filter.rows();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = stateDim - sysStateDim;

  jumpMap.dfdx.resize(postJumpStateDim, stateDim);
  jumpMap.dfdx.topLeftCorner(postJumpSysStateDim, sysStateDim) = jumpMap_system.dfdx;
  jumpMap.dfdx.bottomLeftCorner(postJumpFiltStateDim, sysStateDim).setZero();
  jumpMap.dfdx.topRightCorner(postJumpSysStateDim, filtStateDim).setZero();
  jumpMap.dfdx.bottomRightCorner(postJumpFiltStateDim, filtStateDim).setIdentity();

  jumpMap.dfdu.resize(postJumpStateDim, 0);

  return jumpMap;
}

VectorFunctionLinearApproximation LoopshapingDynamics::GuardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  throw std::runtime_error("[LoopshapingDynamics] Guard surfaces not implemented");
}

vector_t LoopshapingDynamics::FlowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  const vector_t system_df = systemDynamics_->flowMapDerivativeTime(t, x, u);
  const vector_t filter_df = vector_t::Zero(loopshapingDefinition_->getInputFilter().getNumStates());
  return loopshapingDefinition_->concatenateSystemAndFilterState(system_df, filter_df);
}

vector_t LoopshapingDynamics::JumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  const vector_t system_dg = systemDynamics_->jumpMapDerivativeTime(t, x, u);
  const vector_t filter_dg = vector_t::Zero(loopshapingDefinition_->getInputFilter().getNumStates());
  return loopshapingDefinition_->concatenateSystemAndFilterState(system_dg, filter_dg);
}

vector_t LoopshapingDynamics::GuardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  throw std::runtime_error("[LoopshapingDynamics] Guard surfaces not implemented");
}

std::unique_ptr<LoopshapingDynamics> LoopshapingDynamics::Create(const SystemDynamicsBase& systemDynamics,
                                                                 std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  // wrap the system pre-computation
  LoopshapingPreComputation preComputation(systemDynamics.getPreComputation(), loopshapingDefinition);

  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::make_unique<LoopshapingDynamicsOutputPattern>(systemDynamics, std::move(loopshapingDefinition), preComputation);
    case LoopshapingType::eliminatepattern:
      return std::make_unique<LoopshapingDynamicsEliminatePattern>(systemDynamics, std::move(loopshapingDefinition), preComputation);
    default:
      throw std::runtime_error("[LoopshapingDynamics::Create] invalid loopshaping type");
  }
}

}  // namespace bolza
