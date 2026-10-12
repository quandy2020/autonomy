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

#include "bolza/oc/multiple_shooting/projection_multiplier_coefficients.hpp"

namespace bolza {
namespace multiple_shooting {

void ProjectionMultiplierCoefficients::compute(const ScalarFunctionQuadraticApproximation& cost,
                                               const VectorFunctionLinearApproximation& dynamics,
                                               const VectorFunctionLinearApproximation& constraintProjection,
                                               const matrix_t& pseudoInverse) {
  vector_t semiprojectedCost_dfdu = cost.dfdu;
  semiprojectedCost_dfdu.noalias() += cost.dfduu * constraintProjection.f;

  matrix_t semiprojectedCost_dfdux = cost.dfdux;
  semiprojectedCost_dfdux.noalias() += cost.dfduu * constraintProjection.dfdx;

  const matrix_t semiprojectedCost_dfduu = cost.dfduu * constraintProjection.dfdu;

  this->dfdx.noalias() = -pseudoInverse * semiprojectedCost_dfdux;
  this->dfdu.noalias() = -pseudoInverse * semiprojectedCost_dfduu;
  this->dfdcostate.noalias() = -pseudoInverse * dynamics.dfdu.transpose();
  this->f.noalias() = -pseudoInverse * semiprojectedCost_dfdu;
}

}  // namespace multiple_shooting
}  // namespace bolza
