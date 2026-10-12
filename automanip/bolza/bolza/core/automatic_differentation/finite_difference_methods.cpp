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

#include <algorithm>

#include <bolza/core/automatic_differentiation/finite_difference_methods.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FiniteDifferenceDerivative(std::function<vector_t(const vector_t&)> f, const vector_t& x0, scalar_t eps,
                                    bool doubleSidedDerivative) {
  const vector_t f0 = f(x0);
  const size_t varDim = x0.rows();
  const size_t stateDim = f0.rows();
  matrix_t jacobian(stateDim, varDim);

  for (size_t i = 0; i < varDim; i++) {
    // inspired from: http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
    scalar_t h = eps * std::max(fabs(x0(i)), 1.0);

    vector_t xPlus = x0;
    xPlus(i) += h;

    if (doubleSidedDerivative) {
      vector_t xMinus = x0;
      xMinus(i) -= h;
      jacobian.col(i) = (f(xPlus) - f(xMinus)) / (2.0 * h);
    } else {
      jacobian.col(i) = (f(xPlus) - f0) / h;
    }
  }

  return jacobian;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FiniteDifferenceDerivativeState(ControlledSystemBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t eps,
                                         bool doubleSidedDerivative, bool isSecondOrderSystem) {
  auto f = [&](const vector_t& var) -> vector_t { return system.ComputeFlowMap(t, var, u); };

  matrix_t A = FiniteDifferenceDerivative(f, x, eps, doubleSidedDerivative);

  if (isSecondOrderSystem) {
    // Assumes state vector = [x, x_dot]
    A.topLeftCorner(x.rows() / 2, x.rows() / 2).setZero();
    A.topRightCorner(x.rows() / 2, x.rows() / 2).setIdentity();
  }
  return A;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t FiniteDifferenceDerivativeInput(ControlledSystemBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t eps,
                                         bool doubleSidedDerivative, bool isSecondOrderSystem) {
  auto f = [&](const vector_t& var) -> vector_t { return system.ComputeFlowMap(t, x, var); };

  matrix_t B = FiniteDifferenceDerivative(f, u, eps, doubleSidedDerivative);

  if (isSecondOrderSystem) {
    // Assumes state vector = [x, x_dot]
    B.topRows(x.rows() / 2).setZero();
  }
  return B;
}

}  // namespace bolza
