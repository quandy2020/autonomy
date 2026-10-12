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

#include <gtest/gtest.h>

#include "bolza/oc/approximate_model/change_of_input_variables.hpp"
#include "helpers/problems_generation_test.hpp"

using namespace bolza;

namespace {
scalar_t Evaluate(const ScalarFunctionQuadraticApproximation& scalarFunctionQuadraticApproximation, const vector_t& dx,
                  const vector_t& du) {
  // Short hand
  const auto& Q = scalarFunctionQuadraticApproximation.dfdxx;
  const auto& R = scalarFunctionQuadraticApproximation.dfduu;
  const auto& P = scalarFunctionQuadraticApproximation.dfdux;
  const auto& c = scalarFunctionQuadraticApproximation.f;

  vector_t q = scalarFunctionQuadraticApproximation.dfdx;
  q.noalias() += 0.5 * Q * dx;
  q.noalias() += P.transpose() * du;

  vector_t r = scalarFunctionQuadraticApproximation.dfdu;
  r.noalias() += 0.5 * R * du;

  return c + q.dot(dx) + r.dot(du);
}

vector_t Evaluate(const VectorFunctionLinearApproximation& vectorFunctionLinearApproximation, const vector_t& dx, const vector_t& du) {
  // Short hand
  const auto& A = vectorFunctionLinearApproximation.dfdx;
  const auto& B = vectorFunctionLinearApproximation.dfdu;

  vector_t b = vectorFunctionLinearApproximation.f;
  b.noalias() += A * dx;
  b.noalias() += B * du;
  return b;
}
}  // namespace

TEST(quadratic_change_of_input_variables, noPx_noU0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const auto quadratic = GetRandomCost(n, m);

  // Apply change of variables
  auto quadraticProjected = quadratic;
  ChangeOfInputVariables(quadraticProjected, Pu);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const scalar_t unprojected = Evaluate(quadratic, dx, Pu * du_tilde);
  const scalar_t projected = Evaluate(quadraticProjected, dx, du_tilde);
  ASSERT_DOUBLE_EQ(unprojected, projected);
}

TEST(quadratic_change_of_input_variables, withPx) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const matrix_t Px = matrix_t::Random(m, n);
  const auto quadratic = GetRandomCost(n, m);

  // Apply change of variables
  auto quadraticProjected = quadratic;
  ChangeOfInputVariables(quadraticProjected, Pu, Px);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const scalar_t unprojected = Evaluate(quadratic, dx, Pu * du_tilde + Px * dx);
  const scalar_t projected = Evaluate(quadraticProjected, dx, du_tilde);
  ASSERT_DOUBLE_EQ(unprojected, projected);
}

TEST(quadratic_change_of_input_variables, withU0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const vector_t u0 = vector_t::Random(m);
  const auto quadratic = GetRandomCost(n, m);

  // Apply change of variables
  auto quadraticProjected = quadratic;
  ChangeOfInputVariables(quadraticProjected, Pu, matrix_t(), u0);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const scalar_t unprojected = Evaluate(quadratic, dx, Pu * du_tilde + u0);
  const scalar_t projected = Evaluate(quadraticProjected, dx, du_tilde);
  ASSERT_DOUBLE_EQ(unprojected, projected);
}

TEST(quadratic_change_of_input_variables, bothPx_U0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const matrix_t Px = matrix_t::Random(m, n);
  const vector_t u0 = vector_t::Random(m);
  const auto quadratic = GetRandomCost(n, m);

  // Apply change of variables
  auto quadraticProjected = quadratic;
  ChangeOfInputVariables(quadraticProjected, Pu, Px, u0);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const scalar_t unprojected = Evaluate(quadratic, dx, Pu * du_tilde + Px * dx + u0);
  const scalar_t projected = Evaluate(quadraticProjected, dx, du_tilde);
  ASSERT_DOUBLE_EQ(unprojected, projected);
}

TEST(linear_change_of_input_variables, noPx_noU0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const auto linear = GetRandomDynamics(n, m);

  // Apply change of variables
  auto linearProjected = linear;
  ChangeOfInputVariables(linearProjected, Pu);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const vector_t unprojected = Evaluate(linear, dx, Pu * du_tilde);
  const vector_t projected = Evaluate(linearProjected, dx, du_tilde);
  ASSERT_TRUE(unprojected.isApprox(projected));
}

TEST(linear_change_of_input_variables, withPx) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const matrix_t Px = matrix_t::Random(m, n);
  const auto linear = GetRandomDynamics(n, m);

  // Apply change of variables
  auto linearProjected = linear;
  ChangeOfInputVariables(linearProjected, Pu, Px);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const vector_t unprojected = Evaluate(linear, dx, Pu * du_tilde + Px * dx);
  const vector_t projected = Evaluate(linearProjected, dx, du_tilde);
  ASSERT_TRUE(unprojected.isApprox(projected));
}

TEST(linear_change_of_input_variables, withU0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const vector_t u0 = vector_t::Random(m);
  const auto linear = GetRandomDynamics(n, m);

  // Apply change of variables
  auto linearProjected = linear;
  ChangeOfInputVariables(linearProjected, Pu, matrix_t(), u0);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const vector_t unprojected = Evaluate(linear, dx, Pu * du_tilde + u0);
  const vector_t projected = Evaluate(linearProjected, dx, du_tilde);
  ASSERT_TRUE(unprojected.isApprox(projected));
}

TEST(linear_change_of_input_variables, bothPx_U0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const matrix_t Px = matrix_t::Random(m, n);
  const vector_t u0 = vector_t::Random(m);
  const auto linear = GetRandomDynamics(n, m);

  // Apply change of variables
  auto linearProjected = linear;
  ChangeOfInputVariables(linearProjected, Pu, Px, u0);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const vector_t unprojected = Evaluate(linear, dx, Pu * du_tilde + Px * dx + u0);
  const vector_t projected = Evaluate(linearProjected, dx, du_tilde);
  ASSERT_TRUE(unprojected.isApprox(projected));
}