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

#include <bolza/core/loopshaping/dynamics/loopshaping_dynamics_eliminate_pattern.hpp>

namespace bolza {

vector_t LoopshapingDynamicsEliminatePattern::FilterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) {
  const auto& s_filter = loopshapingDefinition_->GetInputFilter();
  if (loopshapingDefinition_->isDiagonal()) {
    return s_filter.GetAdiag().diagonal().cwiseProduct(x_filter) + s_filter.GetBdiag().diagonal().cwiseProduct(u_filter);
  } else {
    vector_t dynamics_filter = s_filter.GetA() * x_filter;
    dynamics_filter.noalias() += s_filter.GetB() * u_filter;
    return dynamics_filter;
  }
}

VectorFunctionLinearApproximation LoopshapingDynamicsEliminatePattern::LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                           const PreComputation& preComp) {
  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& s_filter = loopshapingDefinition_->GetInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.GetSystemState();
  const auto& u_system = preCompLS.GetSystemInput();
  const auto& x_filter = preCompLS.GetFilterState();
  const auto& u_filter = preCompLS.GetFilteredInput();
  const auto dynamics_system = systemDynamics_->LinearApproximation(t, x_system, u_system, preCompLS.GetSystemPreComputation());

  const auto stateDim = x.rows();
  const auto inputDim = u.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x_filter.rows();

  VectorFunctionLinearApproximation dynamics;
  dynamics.f = loopshapingDefinition_->ConcatenateSystemAndFilterState(dynamics_system.f, FilterFlowmap(x_filter, u_filter, u_system));

  dynamics.dfdx.resize(stateDim, stateDim);
  dynamics.dfdx.topLeftCorner(sysStateDim, sysStateDim) = dynamics_system.dfdx;
  dynamics.dfdx.bottomLeftCorner(filtStateDim, sysStateDim).setZero();
  if (isDiagonal) {
    dynamics.dfdx.topRightCorner(sysStateDim, filtStateDim).noalias() = dynamics_system.dfdu * s_filter.GetCdiag();
  } else {
    dynamics.dfdx.topRightCorner(sysStateDim, filtStateDim).noalias() = dynamics_system.dfdu * s_filter.GetC();
  }
  dynamics.dfdx.bottomRightCorner(filtStateDim, filtStateDim) = s_filter.GetA();

  dynamics.dfdu.resize(stateDim, inputDim);
  if (isDiagonal) {
    dynamics.dfdu.topRows(sysStateDim).noalias() = dynamics_system.dfdu * s_filter.GetDdiag();
  } else {
    dynamics.dfdu.topRows(sysStateDim).noalias() = dynamics_system.dfdu * s_filter.GetD();
  }
  dynamics.dfdu.bottomRows(filtStateDim) = s_filter.GetB();

  return dynamics;
}

}  // namespace bolza
