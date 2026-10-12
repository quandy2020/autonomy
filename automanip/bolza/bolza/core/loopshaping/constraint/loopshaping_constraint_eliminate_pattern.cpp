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
#include <bolza/core/loopshaping/constraint/loopshaping_constraint_eliminate_pattern.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LoopshapingConstraintEliminatePattern::GetLinearApproximation(scalar_t t, const vector_t& x,
                                                                                                const vector_t& u,
                                                                                                const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionLinearApproximation::Zero(0, x.rows(), u.rows());
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto stateDim = x.rows();
  const auto sysStateDim = x_system.rows();
  const auto filtStateDim = s_filter.getNumStates();

  // Not const so we can move
  auto g_system = StateInputConstraintCollection::GetLinearApproximation(t, x_system, u_system, preComp_system);
  const auto numConstraints = g_system.f.rows();

  VectorFunctionLinearApproximation g;
  g.f = std::move(g_system.f);

  // dfdx
  g.dfdx.resize(numConstraints, stateDim);
  g.dfdx.leftCols(sysStateDim) = g_system.dfdx;
  if (isDiagonal) {
    g.dfdx.rightCols(filtStateDim).noalias() = g_system.dfdu * s_filter.getCdiag();
  } else {
    g.dfdx.rightCols(filtStateDim).noalias() = g_system.dfdu * s_filter.getC();
  }

  // dfdu
  if (isDiagonal) {
    g.dfdu.noalias() = g_system.dfdu * s_filter.getDdiag();
  } else {
    g.dfdu.noalias() = g_system.dfdu * s_filter.getD();
  }

  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation LoopshapingConstraintEliminatePattern::GetQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                      const vector_t& u,
                                                                                                      const PreComputation& preComp) const {
  if (this->empty()) {
    return VectorFunctionQuadraticApproximation::Zero(0, x.rows(), u.rows());
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& preComp_system = preCompLS.getSystemPreComputation();
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
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
  if (isDiagonal) {
    h.dfdx.rightCols(filtStateDim).noalias() = h_system.dfdu * s_filter.getCdiag();
    h.dfdu.noalias() = h_system.dfdu * s_filter.getDdiag();
  } else {
    h.dfdx.rightCols(filtStateDim).noalias() = h_system.dfdu * s_filter.getC();
    h.dfdu.noalias() = h_system.dfdu * s_filter.getD();
  }

  h.dfdxx.resize(numConstraints);
  h.dfduu.resize(numConstraints);
  h.dfdux.resize(numConstraints);

  if (isDiagonal) {
    for (size_t i = 0; i < numConstraints; i++) {
      // dfdxx
      h.dfdxx[i].resize(stateDim, stateDim);
      h.dfdxx[i].topLeftCorner(sysStateDim, sysStateDim) = h_system.dfdxx[i];
      h.dfdxx[i].bottomLeftCorner(filtStateDim, sysStateDim).noalias() = s_filter.getCdiag() * h_system.dfdux[i];
      h.dfdxx[i].topRightCorner(sysStateDim, filtStateDim) = h.dfdxx[i].bottomLeftCorner(filtStateDim, sysStateDim).transpose();
      h.dfdxx[i].bottomRightCorner(filtStateDim, filtStateDim) = s_filter.getScalingCdiagCdiag().cwiseProduct(h_system.dfduu[i]);

      // dfduu
      h.dfduu[i] = s_filter.getScalingDdiagDdiag().cwiseProduct(h_system.dfduu[i]);

      // dfdux
      h.dfdux[i].resize(inputDim, stateDim);
      h.dfdux[i].leftCols(sysStateDim).noalias() = s_filter.getDdiag() * h_system.dfdux[i];
      h.dfdux[i].rightCols(filtStateDim) = s_filter.getScalingDdiagCdiag().cwiseProduct(h_system.dfduu[i]);
    }

    return h;
  } else {
    matrix_t dfduu_C;  // temporary variable
    for (size_t i = 0; i < numConstraints; i++) {
      // dfdxx
      h.dfdxx[i].resize(stateDim, stateDim);
      h.dfdxx[i].topLeftCorner(sysStateDim, sysStateDim) = h_system.dfdxx[i];
      h.dfdxx[i].bottomLeftCorner(filtStateDim, sysStateDim) = s_filter.getC().transpose() * h_system.dfdux[i];
      h.dfdxx[i].topRightCorner(sysStateDim, filtStateDim).noalias() = h.dfdxx[i].bottomLeftCorner(filtStateDim, sysStateDim).transpose();
      dfduu_C.noalias() = h_system.dfduu[i] * s_filter.getC();
      h.dfdxx[i].bottomRightCorner(filtStateDim, filtStateDim).noalias() = s_filter.getC().transpose() * dfduu_C;

      // dfduu
      h.dfduu[i].noalias() = s_filter.getD().transpose() * h_system.dfduu[i] * s_filter.getD();

      // dfdux
      h.dfdux[i].resize(inputDim, stateDim);
      h.dfdux[i].leftCols(sysStateDim).noalias() = s_filter.getD().transpose() * h_system.dfdux[i];
      h.dfdux[i].rightCols(filtStateDim).noalias() = s_filter.getD().transpose() * dfduu_C;
    }

    return h;
  }
}

}  // namespace bolza
