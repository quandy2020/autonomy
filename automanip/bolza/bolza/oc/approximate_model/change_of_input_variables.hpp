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

#pragma once

/**
 * @file change_of_input_variables.hpp
 * @brief Applies the following change of input variables to the quadraticApproximation (stateDim=n, inputDim=m):.
 */


#include <bolza/core/types.hpp>

namespace bolza {

/**
 * Applies the following change of input variables to the quadraticApproximation (stateDim=n, inputDim=m):
 * \delta u = Pu * \tilde{\delta u} + Px * \delta x + \delta u_0,
 * with sizes Pu (m x p), Px (m x n), u0 (m x 1)
 *
 * The altered model data will be of size stateDim=n, inputDim=p
 *
 * A Px / u0 of zeros can be efficiently applied by passing an empty matrix / vector (of size 0).
 *
 * @param [in] quadraticApproximation: Approximation to be adapted in-place
 * @param [in] Pu: Matrix defining the range of \tilde{\delta u}
 * @param [in] Px: Matrix defining the range of \delta x
 * @param [in] u0: Input offset
 */
void ChangeOfInputVariables(ScalarFunctionQuadraticApproximation& quadraticApproximation, const matrix_t& Pu,
                            const matrix_t& Px = matrix_t(), const vector_t& u0 = vector_t());

/**
 * @brief Applies the change of input variables to a linear system.
 * @param [in,out] linearApproximation: Linear Approximation.
 * @param [in] Pu: Pu.
 * @param [in] Px: Px.
 * @param [in] u0: U0.
 */
void ChangeOfInputVariables(VectorFunctionLinearApproximation& linearApproximation, const matrix_t& Pu, const matrix_t& Px = matrix_t(),
                            const vector_t& u0 = vector_t());

}  // namespace bolza