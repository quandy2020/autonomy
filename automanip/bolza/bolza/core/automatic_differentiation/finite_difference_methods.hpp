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
 * @file finite_difference_methods.hpp
 * @brief finite difference methods (bolza).
 */


#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/controlled_system_base.hpp>

namespace bolza {

/**
 * @brief Finite Difference Derivative.
 * @param [in] f: F.
 * @param [in] x0: X0.
 * @param [in] eps: Eps.
 * @param [in] doubleSidedDerivative: Double Sided Derivative.
 * @return Result value.
 */
matrix_t FiniteDifferenceDerivative(std::function<vector_t(const vector_t&)> f, const vector_t& x0,
                                    scalar_t eps = Eigen::NumTraits<scalar_t>::epsilon(), bool doubleSidedDerivative = true);

/**
 * @brief Finite Difference Derivative State.
 * @param [in,out] system: System.
 * @param [in] t: T.
 * @param [in] x: X.
 * @param [in] u: U.
 * @param [in] eps: Eps.
 * @param [in] doubleSidedDerivative: Double Sided Derivative.
 * @param [in] isSecondOrderSystem: Is Second Order System.
 * @return Result value.
 */
matrix_t FiniteDifferenceDerivativeState(ControlledSystemBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                         scalar_t eps = Eigen::NumTraits<scalar_t>::epsilon(), bool doubleSidedDerivative = true,
                                         bool isSecondOrderSystem = false);

/**
 * @brief Finite Difference Derivative Input.
 * @param [in,out] system: System.
 * @param [in] t: T.
 * @param [in] x: X.
 * @param [in] u: U.
 * @param [in] eps: Eps.
 * @param [in] doubleSidedDerivative: Double Sided Derivative.
 * @param [in] isSecondOrderSystem: Is Second Order System.
 * @return Result value.
 */
matrix_t FiniteDifferenceDerivativeInput(ControlledSystemBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                         scalar_t eps = Eigen::NumTraits<scalar_t>::epsilon(), bool doubleSidedDerivative = true,
                                         bool isSecondOrderSystem = false);
}  // namespace bolza
