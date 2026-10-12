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

#include <bolza/core/loopshaping/loopshaping_pre_computation.hpp>
#include <bolza/core/loopshaping/cost/loopshaping_state_cost.hpp>

namespace bolza {

scalar_t LoopshapingStateCost::GetValue(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories,
                                        const PreComputation& preComp) const {
  if (this->empty()) {
    return 0.0;
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.GetSystemState();

  return StateCostCollection::GetValue(t, x_system, targetTrajectories, preCompLS.GetSystemPreComputation());
}

ScalarFunctionQuadraticApproximation LoopshapingStateCost::GetQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                     const TargetTrajectories& targetTrajectories,
                                                                                     const PreComputation& preComp) const {
  if (this->empty()) {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows());
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.GetSystemState();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x.rows() - sysStateDim;

  const auto Phi_system =
      StateCostCollection::GetQuadraticApproximation(t, x_system, targetTrajectories, preCompLS.GetSystemPreComputation());

  ScalarFunctionQuadraticApproximation Phi;
  Phi.f = Phi_system.f;
  Phi.dfdx.resize(stateDim);
  Phi.dfdx.head(sysStateDim) = Phi_system.dfdx;
  Phi.dfdx.tail(filtStateDim).setZero();
  Phi.dfdxx.setZero(stateDim, stateDim);
  Phi.dfdxx.topLeftCorner(sysStateDim, sysStateDim) = Phi_system.dfdxx;
  return Phi;
}

}  // namespace bolza
