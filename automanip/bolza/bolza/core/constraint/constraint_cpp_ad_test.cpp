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

#include <bolza/core/constraint/state_constraint_cpp_ad.hpp>
#include <bolza/core/constraint/state_input_constraint_cpp_ad.hpp>

class TestStateConstraint : public bolza::StateConstraintCppAd {
 public:
  TestStateConstraint() : bolza::StateConstraintCppAd(bolza::ConstraintOrder::Quadratic) {
    Initialize(2, 0, "TestStateConstraint", "/tmp/bolza", true, false);
  }
  ~TestStateConstraint() override = default;
  TestStateConstraint* Clone() const override { return new TestStateConstraint(*this); }

  size_t GetNumConstraints(bolza::scalar_t time) const override { return 2; }

  bolza::ad_vector_t ConstraintFunction(bolza::ad_scalar_t time, const bolza::ad_vector_t& state,
                                       const bolza::ad_vector_t& parameters) const override {
    bolza::ad_vector_t Constraint(2);
    bolza::matrix_t Q = (bolza::matrix_t(2, 2) << 1.0, 0.0, 0.0, 2.0).finished();
    Constraint(0) = bolza::ad_scalar_t(0.5) * state.dot(Q.cast<bolza::ad_scalar_t>() * state);
    Constraint(1) = 3.0 * state(0) + 4.0 * state(1);
    return constraint;
  }

 private:
  TestStateConstraint(const TestStateConstraint& other) = default;
};

TEST(TestStateConstraintCppAd, getValue) {
  TestStateConstraint constraint;

  const bolza::scalar_t t = 0.0;
  const bolza::vector_t x = bolza::vector_t::Ones(2);

  const auto val = constraint.GetValue(t, x, bolza::PreComputation());
  const auto lin = constraint.GetLinearApproximation(t, x, bolza::PreComputation());
  const auto quad = constraint.GetQuadraticApproximation(t, x, bolza::PreComputation());

  EXPECT_TRUE(val.isApprox((bolza::vector_t(2) << 1.5, 7).finished()));
  EXPECT_TRUE(lin.f.isApprox(val));
  EXPECT_TRUE(lin.dfdx.isApprox((bolza::matrix_t(2, 2) << 1, 2, 3, 4).finished()));
  EXPECT_TRUE(quad.f.isApprox(lin.f));
  EXPECT_TRUE(quad.dfdx.isApprox(lin.dfdx));
  EXPECT_TRUE(quad.dfdxx[0].isApprox((bolza::matrix_t(2, 2) << 1, 0, 0, 2).finished()));
  EXPECT_TRUE(quad.dfdxx[1].isZero());
}

class TestStateInputConstraint : public bolza::StateInputConstraintCppAd {
 public:
  TestStateInputConstraint() : bolza::StateInputConstraintCppAd(bolza::ConstraintOrder::Quadratic) {
    Initialize(2, 1, 0, "TestStateInputConstraint", "/tmp/bolza", true, false);
  }
  ~TestStateInputConstraint() override = default;
  TestStateInputConstraint* Clone() const override { return new TestStateInputConstraint(*this); }

  size_t GetNumConstraints(bolza::scalar_t time) const override { return 2; }

  bolza::ad_vector_t ConstraintFunction(bolza::ad_scalar_t time, const bolza::ad_vector_t& state, const bolza::ad_vector_t& input,
                                       const bolza::ad_vector_t& parameters) const override {
    bolza::ad_vector_t Constraint(2);
    bolza::matrix_t Q = (bolza::matrix_t(2, 2) << 1.0, 0.0, 0.0, 2.0).finished();
    Constraint(0) = bolza::ad_scalar_t(0.5) * state.dot(Q.cast<bolza::ad_scalar_t>() * state) + Input(0);
    Constraint(1) = 3.0 * state(0) + 4.0 * state(1) - Input(0) * Input(0);
    return constraint;
  }

 private:
  TestStateInputConstraint(const TestStateInputConstraint& other) = default;
};

TEST(TestStateInputConstraintCppAd, getValue) {
  TestStateInputConstraint constraint;

  const bolza::scalar_t t = 0.0;
  const bolza::vector_t x = bolza::vector_t::Ones(2);
  const bolza::vector_t u = bolza::vector_t::Ones(1);

  const auto val = constraint.GetValue(t, x, u, bolza::PreComputation());
  const auto lin = constraint.GetLinearApproximation(t, x, u, bolza::PreComputation());
  const auto quad = constraint.GetQuadraticApproximation(t, x, u, bolza::PreComputation());

  EXPECT_TRUE(val.isApprox((bolza::vector_t(2) << 2.5, 6).finished()));
  EXPECT_TRUE(lin.f.isApprox(val));
  EXPECT_TRUE(lin.dfdx.isApprox((bolza::matrix_t(2, 2) << 1, 2, 3, 4).finished()));
  EXPECT_TRUE(lin.dfdu.isApprox((bolza::matrix_t(2, 1) << 1, -2).finished()));
  EXPECT_TRUE(quad.f.isApprox(lin.f));
  EXPECT_TRUE(quad.dfdx.isApprox(lin.dfdx));
  EXPECT_TRUE(quad.dfdu.isApprox(lin.dfdu));
  EXPECT_TRUE(quad.dfdxx[0].isApprox((bolza::matrix_t(2, 2) << 1, 0, 0, 2).finished()));
  EXPECT_TRUE(quad.dfdxx[1].isZero());
  EXPECT_TRUE(quad.dfdux[0].isZero());
  EXPECT_TRUE(quad.dfdux[1].isZero());
  EXPECT_TRUE(quad.dfduu[0].isZero());
  EXPECT_TRUE(quad.dfduu[1].isApprox((bolza::matrix_t(1, 1) << -2).finished()));
}
