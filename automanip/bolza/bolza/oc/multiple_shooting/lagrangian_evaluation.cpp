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

#include "bolza/oc/multiple_shooting/lagrangian_evaluation.hpp"

namespace bolza {
namespace multiple_shooting {

ScalarFunctionQuadraticApproximation EvaluateLagrangianIntermediateNode(const vector_t& lmd, const vector_t& lmd_next, const vector_t& nu,
                                                                        ScalarFunctionQuadraticApproximation&& cost,
                                                                        const VectorFunctionLinearApproximation& dynamics,
                                                                        const VectorFunctionLinearApproximation& stateInputEqConstraints) {
  ScalarFunctionQuadraticApproximation lagrangian = std::move(cost);
  lagrangian.dfdx.noalias() += dynamics.dfdx.transpose() * lmd_next;
  lagrangian.dfdx.noalias() -= lmd;
  lagrangian.dfdu.noalias() += dynamics.dfdu.transpose() * lmd_next;
  if (stateInputEqConstraints.f.size() > 0) {
    lagrangian.dfdx.noalias() += stateInputEqConstraints.dfdx.transpose() * nu;
    lagrangian.dfdu.noalias() += stateInputEqConstraints.dfdu.transpose() * nu;
  }
  return lagrangian;
}

ScalarFunctionQuadraticApproximation EvaluateLagrangianTerminalNode(const vector_t& lmd, ScalarFunctionQuadraticApproximation&& cost) {
  ScalarFunctionQuadraticApproximation lagrangian = std::move(cost);
  lagrangian.dfdx.noalias() -= lmd;
  return lagrangian;
}

ScalarFunctionQuadraticApproximation EvaluateLagrangianEventNode(const vector_t& lmd, const vector_t& lmd_next,
                                                                 ScalarFunctionQuadraticApproximation&& cost,
                                                                 const VectorFunctionLinearApproximation& dynamics) {
  ScalarFunctionQuadraticApproximation lagrangian = std::move(cost);
  lagrangian.dfdx.noalias() += dynamics.dfdx.transpose() * lmd_next;
  lagrangian.dfdx.noalias() -= lmd;
  return lagrangian;
}

}  // namespace multiple_shooting
}  // namespace bolza
