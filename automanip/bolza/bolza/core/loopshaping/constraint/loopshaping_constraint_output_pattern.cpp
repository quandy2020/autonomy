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
#include <bolza/core/loopshaping/constraint/loopshaping_constraint_output_pattern.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LoopshapingConstraintOutputPattern::GetLinearApproximation(scalar_t t, const vector_t& x,
                                                                                             const vector_t& u,
                                                                                             const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionLinearApproximation::Zero(0, x.rows(), u.rows());
  }

  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.GetSystemPreComputation();
  const auto& x_system = preCompLS.GetSystemState();
  const auto& u_system = preCompLS.GetSystemInput();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x.rows() - sysStateDim;

  // Not const so we can move
  auto g_system = StateInputConstraintCollection::GetLinearApproximation(t, x_system, u_system, preComp_system);
  const auto numConstraints = g_system.f.rows();

  VectorFunctionLinearApproximation g;
  g.f = std::move(g_system.f);

  g.dfdx.resize(numConstraints, stateDim);
  g.dfdx.leftCols(sysStateDim) = g_system.dfdx;
  g.dfdx.rightCols(filtStateDim).setZero();

  g.dfdu = std::move(g_system.dfdu);

  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation LoopshapingConstraintOutputPattern::GetQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                   const vector_t& u,
                                                                                                   const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionQuadraticApproximation::Zero(0, x.rows(), u.rows());
  }

  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.GetSystemPreComputation();
  const auto& x_system = preCompLS.GetSystemState();
  const auto& u_system = preCompLS.GetSystemInput();
  const auto& x_filter = preCompLS.GetFilterState();
  const auto stateDim = x.rows();
  const auto inputDim = u.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x_filter.rows();

  // Not const so we can move
  auto h_system = StateInputConstraintCollection::GetQuadraticApproximation(t, x_system, u_system, preComp_system);
  const auto numConstraints = h_system.f.rows();

  VectorFunctionQuadraticApproximation h;
  h.f = std::move(h_system.f);

  h.dfdx.resize(numConstraints, stateDim);
  h.dfdx.leftCols(sysStateDim) = h_system.dfdx;
  h.dfdx.rightCols(filtStateDim).setZero();

  h.dfdu = std::move(h_system.dfdu);

  h.dfdxx.resize(numConstraints);
  h.dfduu.resize(numConstraints);
  h.dfdux.resize(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    h.dfdxx[i].setZero(stateDim, stateDim);
    h.dfdxx[i].topLeftCorner(sysStateDim, sysStateDim) = h_system.dfdxx[i];

    h.dfduu[i] = std::move(h_system.dfduu[i]);

    h.dfdux[i].resize(inputDim, stateDim);
    h.dfdux[i].leftCols(sysStateDim) = h_system.dfdux[i];
    h.dfdux[i].rightCols(filtStateDim).setZero();
  }

  return h;
}

}  // namespace bolza
