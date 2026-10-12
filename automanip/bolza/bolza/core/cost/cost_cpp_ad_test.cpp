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

#include <bolza/core/cost/state_cost_cpp_ad.hpp>
#include <bolza/core/cost/state_input_cost_cpp_ad.hpp>
#include <bolza/core/cost/state_input_gauss_newton_cost_ad.hpp>

class TestStateCost : public bolza::StateCostCppAd {
 public:
  TestStateCost() { Initialize(2, 0, "TestStateCost", "/tmp/bolza", true, false); }
  ~TestStateCost() override = default;
  TestStateCost* Clone() const override { return new TestStateCost(*this); }

  bolza::ad_scalar_t CostFunction(bolza::ad_scalar_t time, const bolza::ad_vector_t& state,
                                 const bolza::ad_vector_t& parameters) const override {
    bolza::ad_scalar_t cost;
    const bolza::matrix_t Q = (bolza::matrix_t(2, 2) << 1.0, 0.0, 0.0, 2.0).finished();
    cost = bolza::ad_scalar_t(0.5) * state.dot(Q.cast<bolza::ad_scalar_t>() * state);
    return cost;
  }

 private:
  TestStateCost(const TestStateCost& other) = default;
};

TEST(TestStateCostCppAd, getValue) {
  TestStateCost cost;
  const bolza::TargetTrajectories desiredTrajectory;

  const bolza::scalar_t t = 0.0;
  const bolza::vector_t x = bolza::vector_t::Ones(2);

  const auto val = cost.GetValue(t, x, desiredTrajectory, bolza::PreComputation());
  const auto approx = cost.GetQuadraticApproximation(t, x, desiredTrajectory, bolza::PreComputation());

  EXPECT_NEAR(val, 1.5, 1e-6);
  EXPECT_NEAR(approx.f, val, 1e-6);
  EXPECT_TRUE(approx.dfdx.isApprox((bolza::matrix_t(2, 1) << 1, 2).finished()));
  EXPECT_TRUE(approx.dfdxx.isApprox((bolza::matrix_t(2, 2) << 1, 0, 0, 2).finished()));
}

class TestStateInputCost : public bolza::StateInputCostCppAd {
 public:
  TestStateInputCost() { Initialize(2, 1, 0, "TestStateInputCost", "/tmp/bolza", true, false); }
  ~TestStateInputCost() override = default;
  TestStateInputCost* Clone() const override { return new TestStateInputCost(*this); }

  bolza::ad_scalar_t CostFunction(bolza::ad_scalar_t time, const bolza::ad_vector_t& state, const bolza::ad_vector_t& input,
                                 const bolza::ad_vector_t& parameters) const override {
    bolza::ad_scalar_t cost;
    const bolza::matrix_t Q = (bolza::matrix_t(2, 2) << 1.0, 0.0, 0.0, 2.0).finished();
    const bolza::matrix_t R = (bolza::matrix_t(1, 1) << 1.0).finished();
    const bolza::matrix_t P = (bolza::matrix_t(1, 2) << 1.0, 1.0).finished();
    cost = bolza::ad_scalar_t(0.5) * state.dot(Q.cast<bolza::ad_scalar_t>() * state);
    cost += bolza::ad_scalar_t(0.5) * input.dot(R.cast<bolza::ad_scalar_t>() * input);
    cost += input.dot(P.cast<bolza::ad_scalar_t>() * state);
    return cost;
  }

 private:
  TestStateInputCost(const TestStateInputCost& other) = default;
};

TEST(TestStateInputCostCppAd, getValue) {
  TestStateInputCost cost;
  const bolza::TargetTrajectories desiredTrajectory;

  const bolza::scalar_t t = 0.0;
  const bolza::vector_t x = bolza::vector_t::Ones(2);
  const bolza::vector_t u = bolza::vector_t::Ones(1);

  const auto val = cost.GetValue(t, x, u, desiredTrajectory, bolza::PreComputation());
  const auto approx = cost.GetQuadraticApproximation(t, x, u, desiredTrajectory, bolza::PreComputation());

  EXPECT_NEAR(val, 4.0, 1e-6);
  EXPECT_NEAR(approx.f, val, 1e-6);
  EXPECT_TRUE(approx.dfdx.isApprox((bolza::matrix_t(2, 1) << 2, 3).finished()));
  EXPECT_TRUE(approx.dfdu.isApprox((bolza::matrix_t(1, 1) << 3).finished()));
  EXPECT_TRUE(approx.dfdxx.isApprox((bolza::matrix_t(2, 2) << 1, 0, 0, 2).finished()));
  EXPECT_TRUE(approx.dfduu.isApprox((bolza::matrix_t(1, 1) << 1).finished()));
  EXPECT_TRUE(approx.dfdux.isApprox((bolza::matrix_t(1, 2) << 1, 1).finished()));
}

class TestGNStateInputCost : public bolza::StateInputCostGaussNewtonAd {
 public:
  TestGNStateInputCost() { Initialize(2, 1, 0, "TestGNStateInputCost", "/tmp/bolza", true, false); }
  ~TestGNStateInputCost() override = default;
  TestGNStateInputCost* Clone() const override { return new TestGNStateInputCost(*this); }

  template <typename SCALAR_T>
  static Eigen::Matrix<SCALAR_T, -1, 1> costVector(SCALAR_T time, const Eigen::Matrix<SCALAR_T, -1, 1>& state,
                                                   const Eigen::Matrix<SCALAR_T, -1, 1>& input) {
    Eigen::Matrix<SCALAR_T, -1, 1> costVector(4);
    costVector << state(0), state(1), time * Input(0), state(0) + Input(0);
    return costVector;
  }

  bolza::ad_vector_t CostVectorFunction(bolza::ad_scalar_t time, const bolza::ad_vector_t& state, const bolza::ad_vector_t& input,
                                       const bolza::ad_vector_t& parameters) const override {
    return costVector(time, state, input);
  }

 private:
  TestGNStateInputCost(const TestGNStateInputCost& other) = default;
};

TEST(TestGNStateInputCostCppAd, getValue) {
  TestGNStateInputCost cost;
  const bolza::TargetTrajectories desiredTrajectory;

  const bolza::scalar_t t = 0.4;
  const bolza::vector_t x = (bolza::vector_t(2) << 0.1, 0.2).finished();
  const bolza::vector_t u = (bolza::vector_t(1) << 0.3).finished();

  bolza::vector_t f = TestGNStateInputCost::costVector(t, x, u);

  const auto val = cost.GetValue(t, x, u, desiredTrajectory, bolza::PreComputation());
  const auto approx = cost.GetQuadraticApproximation(t, x, u, desiredTrajectory, bolza::PreComputation());

  // cost = 0.5 * |f|^2 = 0.5*x(0)^2 + 0.5*x(1)^2 + 0.5*(t*u(0))^2 + 0.5*(x(0) + u(0))^2
  ASSERT_DOUBLE_EQ(val, 0.5 * f.squaredNorm());
  ASSERT_DOUBLE_EQ(approx.f, val);
  ASSERT_DOUBLE_EQ(approx.dfdx(0), 2.0 * x(0) + u(0));
  ASSERT_DOUBLE_EQ(approx.dfdx(1), x(1));
  ASSERT_DOUBLE_EQ(approx.dfdu(0), (t * t + 1.0) * u(0) + x(0));
  ASSERT_DOUBLE_EQ(approx.dfdxx(0, 0), 2.0);
  ASSERT_DOUBLE_EQ(approx.dfdxx(0, 1), 0.0);
  ASSERT_DOUBLE_EQ(approx.dfdxx(1, 0), approx.dfdxx(0, 1));
  ASSERT_DOUBLE_EQ(approx.dfdxx(1, 1), 1.0);
  ASSERT_DOUBLE_EQ(approx.dfdux(0, 0), 1.0);
  ASSERT_DOUBLE_EQ(approx.dfdux(0, 1), 0.0);
  ASSERT_DOUBLE_EQ(approx.dfduu(0, 0), (t * t + 1.0));
}
