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
 * @file cpp_ad_sparsity.hpp
 * @brief Vector contains 1 set per row, each set contains the indices of non-zero columns.
 */


#include <algorithm>
#include <functional>  // missing header in cg.hpp
#include <vector>

#include <cppad/cg.hpp>

namespace bolza {

namespace cppad_sparsity {

/**
 *  Vector contains 1 set per row, each set contains the indices of non-zero columns.
 */
using SparsityPattern = std::vector<std::set<size_t>>;

/**
 * Gets the Jacobian sparsity pattern of a taped CppAD function.
 * @tparam ad_fun_t : CppAD function type.
 * @param [in] fun: function that has been taped already.
 * @return Sparsity pattern.
 */
template <typename ad_fun_t>
/**
 * @brief Get Jacobian Sparsity Pattern.
 * @param [in,out] fun: Fun.
 * @return Result value.
 */
SparsityPattern GetJacobianSparsityPattern(ad_fun_t& fun) {
  return CppAD::cg::jacobianSparsitySet<SparsityPattern>(fun);
}

/**
 * Gets the Hessian sparsity pattern of a taped CppAD function.
 * @tparam ad_fun_t : CppAD function type.
 * @param [in] fun: function that has been taped already.
 * @return Sparsity pattern.
 */
template <typename ad_fun_t>
/**
 * @brief Get Hessian Sparsity Pattern.
 * @param [in,out] fun: Fun.
 * @return Result value.
 */
SparsityPattern GetHessianSparsityPattern(ad_fun_t& fun) {
  return CppAD::cg::hessianSparsitySet<SparsityPattern>(fun);
}

/**
 * Returns a sparsity pattern that has entries only there where both p0 and p1 have entries.
 * @param [in] p0: First sparsity pattern.
 * @param [in] p1: Second sparsity pattern.
 * @return Intersected sparsity pattern.
 */
SparsityPattern GetIntersection(const SparsityPattern& p0, const SparsityPattern& p1);

/**
 * Constructs a Jacobian sparsity pattern that has dense entries for the first variableDim columns of rangeDim rows.
 * @param [in] rangeDim: number of rows to generate dense sparsity for.
 * @param [in] variableDim: number of cols to generate dense sparsity for.
 * @return sparsity pattern.
 */
SparsityPattern GetJacobianVariableSparsity(int rangeDim, int variableDim);

/**
 * Constructs a upper triangular Hessian sparsity pattern that has dense entries for the variableDim x variableDim in the top left corner.
 * @param [in] variableDim: size of the dense uppertriangular block in the upper left corner.
 * @param [in] parameterDim: Additional empty rows to add after the dense block.
 * @return sparsity pattern.
 */
SparsityPattern GetHessianVariableSparsity(int variableDim, int parameterDim);

/**
 * @brief.
 *
 * Get number of nonzeros in sparsity pattern
 *
 * /
 * @param [in] sparsityPattern: Sparsity Pattern.
 * @return number of nonzero elements
 */
size_t GetNumberOfNonZeros(const SparsityPattern& sparsityPattern);

}  // namespace cppad_sparsity
}  // namespace bolza
