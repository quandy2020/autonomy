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
#include <bolza/core/loopshaping/constraint/loopshaping_state_constraint.hpp>

#include <bolza/core/loopshaping/constraint/loopshaping_constraint_eliminate_pattern.hpp>
#include <bolza/core/loopshaping/constraint/loopshaping_constraint_output_pattern.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t LoopshapingStateConstraint::GetValue(scalar_t t, const vector_t& x, const PreComputation& preComp) const {
  if (this->empty()) {
    return vector_array_t();
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.GetSystemState();
  const auto& preComp_system = preCompLS.GetSystemPreComputation();

  return StateConstraintCollection::GetValue(t, x_system, preComp_system);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LoopshapingStateConstraint::GetLinearApproximation(scalar_t t, const vector_t& x,
                                                                                     const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionLinearApproximation::Zero(0, x.rows());
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.GetSystemState();
  const auto& preComp_system = preCompLS.GetSystemPreComputation();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x.rows() - sysStateDim;

  // Not const so we can move
  auto c_system = StateConstraintCollection::GetLinearApproximation(t, x_system, preComp_system);
  const auto numConstraints = c_system.f.rows();

  VectorFunctionLinearApproximation c;

  c.f = std::move(c_system.f);

  c.dfdx.resize(numConstraints, stateDim);
  c.dfdx.leftCols(sysStateDim) = c_system.dfdx;
  c.dfdx.rightCols(filtStateDim).setZero();

  return c;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation LoopshapingStateConstraint::GetQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                           const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionQuadraticApproximation::Zero(0, x.rows());
  }

  const LoopshapingPreComputation& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.GetSystemState();
  const auto& preComp_system = preCompLS.GetSystemPreComputation();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x.rows() - sysStateDim;

  // Not const so we can move
  const auto c_system = StateConstraintCollection::GetQuadraticApproximation(t, x_system, preComp_system);
  const auto numConstraints = c_system.f.rows();

  VectorFunctionQuadraticApproximation c;

  c.f = std::move(c_system.f);

  c.dfdx.resize(numConstraints, stateDim);
  c.dfdx.leftCols(sysStateDim) = c_system.dfdx;
  c.dfdx.rightCols(filtStateDim).setZero();

  c.dfdxx.resize(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    c.dfdxx[i].setZero(stateDim, stateDim);
    c.dfdxx[i].topLeftCorner(sysStateDim, sysStateDim) = c_system.dfdxx[i];
  }

  return c;
}

}  // namespace bolza
