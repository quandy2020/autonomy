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

#include "bolza/oc/approximate_model/change_of_input_variables.hpp"

namespace bolza {

void ChangeOfInputVariables(ScalarFunctionQuadraticApproximation& quadraticApproximation, const matrix_t& Pu, const matrix_t& Px,
                            const vector_t& u0) {
  /*
   * 3 temporaries are needed in any branch because Pu is non-zero and:
   *  - new P contains a product Pu'*P
   *  - new R contains a product R*Pu
   *  - new r contains a product Pu'*r
   *  Other than that, the adaptation can be done in-place
   *
   *  The terms of the quadratic functions have the following notation:
   *  dfdxx = Q, dfdux = P, dfduu = R, dfdx = q, dfdu = r, f = c.
   */
  const bool hasPx(Px.size() > 0);
  const bool hasu0(u0.size() > 0);

  // Shared term number 1
  matrix_t P_plus_R_Px = quadraticApproximation.dfdux;
  if (hasPx) {
    P_plus_R_Px.noalias() += quadraticApproximation.dfduu * Px;
  }  // else added term is zero

  // Shared term number 2
  vector_t r_plus_R_u0 = quadraticApproximation.dfdu;
  if (hasu0) {
    r_plus_R_u0.noalias() += quadraticApproximation.dfduu * u0;
  }  // else added term is zero

  // Q = Q + P'*Px + Px'*P + Px'*R*Px = Q + P'*Px + Px'*(P + R*Px)
  if (hasPx) {
    quadraticApproximation.dfdxx.noalias() += quadraticApproximation.dfdux.transpose() * Px;  // Before adapting dfdux!
    quadraticApproximation.dfdxx.noalias() += Px.transpose() * P_plus_R_Px;
  }  // else Q remains unaltered

  // q = q + P' * u0 + Px' (R*u0 + r)
  if (hasu0) {
    quadraticApproximation.dfdx.noalias() += quadraticApproximation.dfdux.transpose() * u0;  // Before adapting dfdux!
  }
  if (hasPx) {
    quadraticApproximation.dfdx.noalias() += Px.transpose() * r_plus_R_u0;
  }

  // c = c + r'*u0 + 1/2*u0'*R*u0 = 1/2*u0'((R*u0 + r) + r)
  if (hasu0) {
    quadraticApproximation.f += 0.5 * u0.dot(r_plus_R_u0 + quadraticApproximation.dfdu);  // Before adapting dfdu!
  }

  // P = Pu'*P + Pu'*R*Px = Pu'*(P + R*Px)
  quadraticApproximation.dfdux.noalias() = Pu.transpose() * P_plus_R_Px;

  // R = Pu' * R * Pu
  matrix_t R_Pu = quadraticApproximation.dfduu * Pu;  // make the required temporary explicit, to save it in the second multiplication
  quadraticApproximation.dfduu.noalias() = Pu.transpose() * R_Pu;

  // r = Pu' * (R*u0 + r)
  quadraticApproximation.dfdu.noalias() = Pu.transpose() * r_plus_R_u0;
}

void ChangeOfInputVariables(VectorFunctionLinearApproximation& linearApproximation, const matrix_t& Pu, const matrix_t& Px,
                            const vector_t& u0) {
  const bool hasPx(Px.size() > 0);
  const bool hasu0(u0.size() > 0);

  // A = A + B*Px
  if (hasPx) {
    linearApproximation.dfdx.noalias() += linearApproximation.dfdu * Px;  // Before adapting dfdu!
  }

  // b = b + B*u0
  if (hasu0) {
    linearApproximation.f.noalias() += linearApproximation.dfdu * u0;  // Before adapting dfdu!
  }

  // B = B*Pu
  linearApproximation.dfdu = linearApproximation.dfdu * Pu;  // temporary matrix unavoidable
}

}  // namespace bolza
