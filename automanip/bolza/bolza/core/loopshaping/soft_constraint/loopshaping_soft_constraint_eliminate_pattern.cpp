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
#include <bolza/core/loopshaping/soft_constraint/loopshaping_soft_constraint_eliminate_pattern.hpp>

namespace bolza {

ScalarFunctionQuadraticApproximation LoopshapingSoftConstraintEliminatePattern::GetQuadraticApproximation(
    scalar_t t, const vector_t& x, const vector_t& u, const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
  if (this->empty()) {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows(), u.rows());
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& s_filter = loopshapingDefinition_->GetInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.GetSystemState();
  const auto& u_system = preCompLS.GetSystemInput();
  const auto& x_filter = preCompLS.GetFilterState();
  const auto stateDim = x.rows();
  const auto inputDim = u.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x_filter.rows();

  const auto L_system =
      StateInputCostCollection::GetQuadraticApproximation(t, x_system, u_system, targetTrajectories, preCompLS.GetSystemPreComputation());

  ScalarFunctionQuadraticApproximation L;

  // f
  L.f = L_system.f;

  if (isDiagonal) {
    // dfdx
    L.dfdx.resize(stateDim);
    L.dfdx.head(sysStateDim) = L_system.dfdx;
    L.dfdx.tail(filtStateDim) = s_filter.GetCdiag().diagonal().cwiseProduct(L_system.dfdu);

    // dfdxx
    L.dfdxx.resize(stateDim, stateDim);
    L.dfdxx.topLeftCorner(sysStateDim, sysStateDim) = L_system.dfdxx;
    L.dfdxx.bottomLeftCorner(filtStateDim, sysStateDim).noalias() = s_filter.GetCdiag() * L_system.dfdux;
    L.dfdxx.topRightCorner(sysStateDim, filtStateDim) = L.dfdxx.bottomLeftCorner(filtStateDim, sysStateDim).transpose();
    L.dfdxx.bottomRightCorner(filtStateDim, filtStateDim) = s_filter.GetScalingCdiagCdiag().cwiseProduct(L_system.dfduu);

    // dfdu & dfduu
    L.dfdu = s_filter.GetDdiag().diagonal().cwiseProduct(L_system.dfdu);
    L.dfduu = s_filter.GetScalingDdiagDdiag().cwiseProduct(L_system.dfduu);

    // dfdux
    L.dfdux.resize(inputDim, stateDim);
    L.dfdux.leftCols(sysStateDim).noalias() = s_filter.GetDdiag() * L_system.dfdux;
    L.dfdux.rightCols(filtStateDim) = s_filter.GetScalingDdiagCdiag().cwiseProduct(L_system.dfduu);

    return L;
  } else {
    // dfdx
    L.dfdx.resize(stateDim);
    L.dfdx.head(sysStateDim) = L_system.dfdx;
    L.dfdx.tail(filtStateDim).noalias() = s_filter.GetC().transpose() * L_system.dfdu;

    // dfdxx
    L.dfdxx.resize(stateDim, stateDim);
    L.dfdxx.topLeftCorner(sysStateDim, sysStateDim) = L_system.dfdxx;
    L.dfdxx.bottomLeftCorner(filtStateDim, sysStateDim) = s_filter.GetC().transpose() * L_system.dfdux;
    L.dfdxx.topRightCorner(sysStateDim, filtStateDim).noalias() = L.dfdxx.bottomLeftCorner(filtStateDim, sysStateDim).transpose();
    matrix_t dfduu_C = L_system.dfduu * s_filter.GetC();
    L.dfdxx.bottomRightCorner(filtStateDim, filtStateDim).noalias() = s_filter.GetC().transpose() * dfduu_C;

    // dfdu & dfduu
    L.dfdu.noalias() = s_filter.GetD().transpose() * L_system.dfdu;
    L.dfduu.noalias() = s_filter.GetD().transpose() * L_system.dfduu * s_filter.GetD();

    // dfdux
    L.dfdux.resize(inputDim, stateDim);
    L.dfdux.leftCols(sysStateDim).noalias() = s_filter.GetD().transpose() * L_system.dfdux;
    L.dfdux.rightCols(filtStateDim).noalias() = s_filter.GetD().transpose() * dfduu_C;

    return L;
  }
}

}  // namespace bolza
