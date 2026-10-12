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
 * @file hessian_correction.hpp
 * @brief Enum used in selecting either DIAGONAL_SHIFT, CHOLESKY_MODIFICATION, EIGENVALUE_MODIFICATION, or GERSHGORIN_MODIFICATION strategies.
 */


#include <string>

#include <bolza/core/numeric_traits.hpp>
#include <bolza/core/misc/linear_algebra.hpp>

namespace bolza {
namespace hessian_correction {

/**
 * @brief The Hessian matrix correction strategy
 * Enum used in selecting either DIAGONAL_SHIFT, CHOLESKY_MODIFICATION, EIGENVALUE_MODIFICATION, or GERSHGORIN_MODIFICATION strategies.
 */
enum class Strategy { DIAGONAL_SHIFT, CHOLESKY_MODIFICATION, EIGENVALUE_MODIFICATION, GERSHGORIN_MODIFICATION };

/**
 * @brief.
 *
 * Get string name of Hessian_Correction type
 *
 * /
 * @param [in] strategy: Hessian_Correction type enum
 * @return Result value.
 */
std::string ToString(Strategy strategy);

/**
 * @brief.
 *
 * Get Hessian_Correction type from string name, useful for reading config file
 *
 * /
 * @param [in] name: Hessian_Correction name
 * @return Result value.
 */
Strategy FromString(const std::string& name);

/**
 * Shifts the Hessian based on the strategy defined by Line_Search::hessianCorrectionStrategy_.
 *
 * @param [in] strategy: Hessian matrix correction strategy.
 * @param [in,out] matrix: The Hessian matrix.
 * @param [in] minEigenvalue: The minimum expected eigenvalue after correction.
 */
void ShiftHessian(Strategy strategy, matrix_t& matrix, scalar_t minEigenvalue = numeric_traits::LimitEpsilon<scalar_t>());

}  // namespace hessian_correction
}  // namespace bolza
