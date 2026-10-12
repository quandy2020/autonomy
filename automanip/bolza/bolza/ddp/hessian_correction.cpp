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

#include "bolza/ddp/hessian_correction.hpp"

#include <unordered_map>

namespace bolza {
namespace hessian_correction {

std::string ToString(Strategy strategy) {
  static const std::unordered_map<Strategy, std::string> strategyMap{{Strategy::DIAGONAL_SHIFT, "DIAGONAL_SHIFT"},
                                                                     {Strategy::CHOLESKY_MODIFICATION, "CHOLESKY_MODIFICATION"},
                                                                     {Strategy::EIGENVALUE_MODIFICATION, "EIGENVALUE_MODIFICATION"},
                                                                     {Strategy::GERSHGORIN_MODIFICATION, "GERSHGORIN_MODIFICATION"}};
  return strategyMap.at(strategy);
}

Strategy FromString(const std::string& name) {
  static const std::unordered_map<std::string, Strategy> strategyMap{{"DIAGONAL_SHIFT", Strategy::DIAGONAL_SHIFT},
                                                                     {"CHOLESKY_MODIFICATION", Strategy::CHOLESKY_MODIFICATION},
                                                                     {"EIGENVALUE_MODIFICATION", Strategy::EIGENVALUE_MODIFICATION},
                                                                     {"GERSHGORIN_MODIFICATION", Strategy::GERSHGORIN_MODIFICATION}};
  return strategyMap.at(name);
}

void ShiftHessian(Strategy strategy, matrix_t& matrix, scalar_t minEigenvalue) {
  assert(matrix.rows() == matrix.cols());
  switch (strategy) {
    case Strategy::DIAGONAL_SHIFT: {
      matrix.diagonal().array() += minEigenvalue;
      break;
    }
    case Strategy::CHOLESKY_MODIFICATION: {
      LinearAlgebra::MakePsdCholesky(matrix, minEigenvalue);
      break;
    }
    case Strategy::EIGENVALUE_MODIFICATION: {
      LinearAlgebra::MakePsdEigenvalue(matrix, minEigenvalue);
      break;
    }
    case Strategy::GERSHGORIN_MODIFICATION: {
      LinearAlgebra::MakePsdGershgorin(matrix, minEigenvalue);
      break;
    }
  }
}

}  // namespace hessian_correction
}  // namespace bolza
