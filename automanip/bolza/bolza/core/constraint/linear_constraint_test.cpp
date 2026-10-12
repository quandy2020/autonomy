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

#include <bolza/core/constraint/linear_state_constraint.hpp>
#include <bolza/core/constraint/linear_state_input_constraint.hpp>

TEST(TestLinearConstraint, testLinearStateInputConstraint) {
  const bolza::vector_t e = bolza::vector_t::Random(3);
  const bolza::matrix_t C = bolza::matrix_t::Random(3, 2);
  const bolza::matrix_t D = bolza::matrix_t::Random(3, 1);
  bolza::LinearStateInputConstraint Constraint(e, C, D);

  const bolza::scalar_t t = 0.0;
  const bolza::vector_t x = bolza::vector_t::Random(2);
  const bolza::vector_t u = bolza::vector_t::Random(1);

  const auto value = constraint.GetValue(t, x, u, bolza::PreComputation());
  const auto approx = constraint.GetLinearApproximation(t, x, u, bolza::PreComputation());
  EXPECT_EQ(constraint.GetNumConstraints(t), value.rows());
  EXPECT_TRUE(value.isApprox(C * x + D * u + e));
  EXPECT_TRUE(approx.f.isApprox(value));
  EXPECT_TRUE(approx.dfdx.isApprox(C));
  EXPECT_TRUE(approx.dfdu.isApprox(D));
}

TEST(TestLinearConstraint, testLinearStateConstraint) {
  const bolza::vector_t e = bolza::vector_t::Random(3);
  const bolza::matrix_t C = bolza::matrix_t::Random(3, 2);
  bolza::LinearStateConstraint Constraint(e, C);

  const bolza::scalar_t t = 0.0;
  const bolza::vector_t x = bolza::vector_t::Random(2);

  const auto value = constraint.GetValue(t, x, bolza::PreComputation());
  const auto approx = constraint.GetLinearApproximation(t, x, bolza::PreComputation());
  EXPECT_EQ(constraint.GetNumConstraints(t), value.rows());
  EXPECT_TRUE(value.isApprox(C * x + e));
  EXPECT_TRUE(approx.f.isApprox(value));
  EXPECT_TRUE(approx.dfdx.isApprox(C));
}
