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

#include <bolza/core/automatic_differentiation/cpp_ad_sparsity.hpp>

namespace bolza {

namespace cppad_sparsity {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SparsityPattern GetIntersection(const SparsityPattern& p0, const SparsityPattern& p1) {
  assert(p0.size() == p1.size());
  const auto numRows = p0.size();

  SparsityPattern result(p0.size());
  for (int row = 0; row < numRows; row++) {
    std::set_intersection(p0[row].begin(), p0[row].end(), p1[row].begin(), p1[row].end(), std::inserter(result[row], result[row].begin()));
  }
  return result;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SparsityPattern GetJacobianVariableSparsity(int rangeDim, int variableDim) {
  // Jacobian : all variables are declared non-zero
  SparsityPattern jacobianSparsity(rangeDim);
  for (auto& sparsityRow : jacobianSparsity) {
    for (size_t i = 0; i < variableDim; i++) {
      sparsityRow.insert(i);
    }
  }
  return jacobianSparsity;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SparsityPattern GetHessianVariableSparsity(int variableDim, int parameterDim) {
  // Hessian : all upper triangular variable entries are declared non-zero
  SparsityPattern hessianSparsity(variableDim + parameterDim);
  for (size_t i = 0; i < variableDim; i++) {
    for (size_t j = i; j < variableDim; j++) {
      hessianSparsity[i].insert(j);
    }
  }
  return hessianSparsity;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t GetNumberOfNonZeros(const SparsityPattern& sparsityPattern) {
  size_t nnz = 0;
  for (const auto& row : sparsityPattern) {
    nnz += row.size();
  }
  return nnz;
}

}  // namespace cppad_sparsity
}  // namespace bolza
