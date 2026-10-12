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

#include <cmath>
#include <ostream>

#include <bolza/core/types.hpp>

namespace bolza {

/** Check for approximate equality of ScalarFunctionQuadraticApproximation */
inline bool IsApprox(const ScalarFunctionQuadraticApproximation& a, const ScalarFunctionQuadraticApproximation& b,
                     scalar_t precision = 1e-9) {
  return std::abs(a.f - b.f) < precision && a.dfdx.isApprox(b.dfdx, precision) && a.dfdu.isApprox(b.dfdu, precision) &&
         a.dfdxx.isApprox(b.dfdxx, precision) && a.dfduu.isApprox(b.dfduu, precision) && a.dfdux.isApprox(b.dfdux, precision);
}

/** Check for approximate equality of VectorFunctionLinearApproximation */
inline bool IsApprox(const VectorFunctionLinearApproximation& a, const VectorFunctionLinearApproximation& b, scalar_t precision = 1e-9) {
  return a.f.isApprox(b.f, precision) && a.dfdx.isApprox(b.dfdx, precision) && a.dfdu.isApprox(b.dfdu, precision);
}

/** Check for approximate equality of VectorFunctionQuadraticApproximation */
inline bool IsApprox(const VectorFunctionQuadraticApproximation& a, const VectorFunctionQuadraticApproximation& b,
                     scalar_t precision = 1e-9) {
  if (!(a.f.isApprox(b.f, precision) && a.dfdx.isApprox(b.dfdx, precision) && a.dfdu.isApprox(b.dfdu, precision))) {
    return false;
  }
  for (size_t i = 0; i < a.f.rows(); i++) {
    if (!(a.dfdxx[i].isApprox(b.dfdxx[i], precision) && a.dfdux[i].isApprox(b.dfdux[i], precision) &&
          a.dfduu[i].isApprox(b.dfduu[i], precision))) {
      return false;
    }
  }
  return true;
}

/**
 * Compares two Eigen vectors on equality.
 * @param tol : tolerance (default value is 1e-12, which is the default of IsApprox().
 */
inline bool IsEqual(const vector_t& lhs, const vector_t& rhs, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  if (lhs.norm() > tol && rhs.norm() > tol) {
    return lhs.isApprox(rhs, tol);
  } else {
    return (lhs - rhs).norm() < tol;
  }
}

/**
 * Compares two scalars on equality in way that is consistent with the check for vectors.
 * @param tol : tolerance (default value is 1e-12, which is the default of IsApprox().
 */
inline bool IsEqual(const scalar_t& lhs, const scalar_t& rhs, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  return IsEqual((vector_t(1) << lhs).finished(), (vector_t(1) << rhs).finished(), tol);
}

/**
 * Compares two Eigen matrices on equality.
 * @param tol : tolerance (default value is 1e-12, which is the default of IsApprox().
 */
inline bool IsEqual(const matrix_t& lhs, const matrix_t& rhs, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  if (lhs.norm() > tol && rhs.norm() > tol) {
    return lhs.isApprox(rhs, tol);
  } else {
    return (lhs - rhs).norm() < tol;
  }
}

/**
 * Compares two trajectories on element-wise approximate equality
 * @param tol : tolerance (default value is 1e-12, which is the default of IsApprox().
 * @return Vectors are of equal length and equal values.
 */
template <typename T>
inline bool IsEqual(const std::vector<T>& v0, const std::vector<T>& v1, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  return (v0.size() == v1.size()) &&
         std::Equal(v0.begin(), v0.end(), v1.begin(), [tol](const T& lhs, const T& rhs) { return IsEqual(lhs, rhs, tol); });
}

}  // namespace bolza
