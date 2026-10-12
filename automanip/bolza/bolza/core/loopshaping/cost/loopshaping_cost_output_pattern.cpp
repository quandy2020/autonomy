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
#include <bolza/core/loopshaping/cost/loopshaping_cost_output_pattern.hpp>

namespace bolza {

ScalarFunctionQuadraticApproximation LoopshapingCostOutputPattern::GetQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                             const vector_t& u,
                                                                                             const TargetTrajectories& targetTrajectories,
                                                                                             const PreComputation& preComp) const {
  if (this->empty()) {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows(), u.rows());
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();
  const auto stateDim = x.rows();
  const auto inputDim = u.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = x_filter.rows();

  const auto& Rfilter = loopshapingDefinition_->costMatrix();
  const vector_t Ru_filter = Rfilter * u_filter;

  // Not const, so we can move
  auto L_system =
      StateInputCostCollection::GetQuadraticApproximation(t, x_system, u_system, targetTrajectories, preCompLS.getSystemPreComputation());

  ScalarFunctionQuadraticApproximation L;
  L.f = L_system.f + 0.5 * u_filter.dot(Ru_filter);

  if (isDiagonal) {
    L.dfdx.resize(stateDim);
    L.dfdx.head(sysStateDim) = L_system.dfdx;
    L.dfdx.tail(filtStateDim) = r_filter.getCdiag().diagonal().cwiseProduct(Ru_filter);

    L.dfdxx.setZero(stateDim, stateDim);
    L.dfdxx.topLeftCorner(sysStateDim, sysStateDim) = L_system.dfdxx;
    L.dfdxx.bottomRightCorner(filtStateDim, filtStateDim) = r_filter.getScalingCdiagCdiag().cwiseProduct(Rfilter);

    L.dfdu = L_system.dfdu + r_filter.getDdiag().diagonal().cwiseProduct(Ru_filter);
    L.dfduu = L_system.dfduu + r_filter.getScalingDdiagDdiag().cwiseProduct(Rfilter);

    L.dfdux.resize(inputDim, stateDim);
    L.dfdux.leftCols(sysStateDim) = L_system.dfdux;
    L.dfdux.rightCols(filtStateDim) = r_filter.getScalingDdiagCdiag().cwiseProduct(Rfilter);

    return L;
  } else {
    L.dfdx.resize(stateDim);
    L.dfdx.head(sysStateDim) = L_system.dfdx;
    L.dfdx.tail(filtStateDim).noalias() = r_filter.getC().transpose() * Ru_filter;

    L.dfdxx.setZero(stateDim, stateDim);
    L.dfdxx.topLeftCorner(sysStateDim, sysStateDim) = L_system.dfdxx;
    const matrix_t dfduu_C = Rfilter * r_filter.getC();
    L.dfdxx.bottomRightCorner(filtStateDim, filtStateDim).noalias() = r_filter.getC().transpose() * dfduu_C;

    L.dfdu = std::move(L_system.dfdu);
    L.dfdu.noalias() += r_filter.getD().transpose() * Ru_filter;
    L.dfduu = std::move(L_system.dfduu);
    L.dfduu.noalias() += r_filter.getD().transpose() * Rfilter * r_filter.getD();

    L.dfdux.resize(inputDim, stateDim);
    L.dfdux.leftCols(sysStateDim) = L_system.dfdux;
    L.dfdux.rightCols(filtStateDim).noalias() = r_filter.getD().transpose() * dfduu_C;

    return L;
  }
}

}  // namespace bolza
