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

#include <bolza/core/cost/state_cost_collection.hpp>
#include <bolza/core/cost/state_input_cost_collection.hpp>

class SimpleQuadraticCost final : public bolza::StateInputCost {
 public:
  SimpleQuadraticCost(bolza::matrix_t Q, bolza::matrix_t R) : Q_(std::move(Q)), R_(std::move(R)) {}
  ~SimpleQuadraticCost() override = default;

  SimpleQuadraticCost* Clone() const override { return new SimpleQuadraticCost(*this); }

  bool IsActive(bolza::scalar_t time) const override { return active_; }

  bolza::scalar_t GetValue(bolza::scalar_t t, const bolza::vector_t& x, const bolza::vector_t& u,
                          const bolza::TargetTrajectories& targetTrajectories, const bolza::PreComputation&) const override {
    return 0.5 * x.dot(Q_ * x) + 0.5 * u.dot(R_ * u);
  }

  bolza::ScalarFunctionQuadraticApproximation GetQuadraticApproximation(bolza::scalar_t t, const bolza::vector_t& x, const bolza::vector_t& u,
                                                                       const bolza::TargetTrajectories& targetTrajectories,
                                                                       const bolza::PreComputation&) const override {
    bolza::ScalarFunctionQuadraticApproximation quadraticApproximation;
    quadraticApproximation.f = 0.5 * x.dot(Q_ * x) + 0.5 * u.dot(R_ * u);
    quadraticApproximation.dfdx = Q_ * x;
    quadraticApproximation.dfdu = R_ * u;
    quadraticApproximation.dfdxx = Q_;
    quadraticApproximation.dfduu = R_;
    quadraticApproximation.dfdux.setZero(u.rows(), x.rows());
    return quadraticApproximation;
  }

  bool active_ = true;

 private:
  bolza::matrix_t Q_;
  bolza::matrix_t R_;
};

class StateInputCost_TestFixture : public ::testing::Test {
 public:
  const size_t STATE_DIM = 4;
  const size_t INPUT_DIM = 2;

  StateInputCost_TestFixture() {
    // Define cost parameters
    bolza::matrix_t Q = bolza::matrix_t::Random(STATE_DIM, STATE_DIM);
    bolza::matrix_t R = bolza::matrix_t::Random(INPUT_DIM, INPUT_DIM);
    Q = (Q + Q.transpose()).eval();
    R = (R + R.transpose()).eval();

    x = bolza::vector_t::Random(STATE_DIM);
    u = bolza::vector_t::Random(INPUT_DIM);
    t = 0.0;

    auto cost1 = std::make_unique<SimpleQuadraticCost>(Q, R);
    auto cost2 = std::make_unique<SimpleQuadraticCost>(0.5 * Q, 2.0 * R);
    expectedCost = cost1->GetValue(t, x, u, targetTrajectories, {}) + cost2->GetValue(t, x, u, targetTrajectories, {});
    expectedCostApproximation = cost1->GetQuadraticApproximation(t, x, u, targetTrajectories, {});
    expectedCostApproximation += cost2->GetQuadraticApproximation(t, x, u, targetTrajectories, {});

    costCollection.Add("Simple quadratic cost", std::move(cost1));
    costCollection.Add("Another simple quadratic cost", std::move(cost2));
  }

  bolza::TargetTrajectories targetTrajectories;
  bolza::StateInputCostCollection costCollection;

  bolza::vector_t x;
  bolza::vector_t u;
  bolza::scalar_t t;
  bolza::scalar_t expectedCost;
  bolza::ScalarFunctionQuadraticApproximation expectedCostApproximation;
};

TEST_F(StateInputCost_TestFixture, getStateInputCost) {
  const auto cost = costCollection.GetValue(t, x, u, targetTrajectories, {});
  EXPECT_NEAR(cost, expectedCost, 1e-6);
}

TEST_F(StateInputCost_TestFixture, getStateInputCostApproximation) {
  const auto cost = costCollection.GetQuadraticApproximation(t, x, u, targetTrajectories, {});
  EXPECT_NEAR(cost.f, expectedCost, 1e-6);
  EXPECT_TRUE(cost.dfdx.isApprox(expectedCostApproximation.dfdx));
  EXPECT_TRUE(cost.dfdu.isApprox(expectedCostApproximation.dfdu));
  EXPECT_TRUE(cost.dfdxx.isApprox(expectedCostApproximation.dfdxx));
  EXPECT_TRUE(cost.dfduu.isApprox(expectedCostApproximation.dfduu));
  EXPECT_TRUE((cost.dfdux.array() == 0.0).all());
}

TEST_F(StateInputCost_TestFixture, canGetCostFunction) {
  const auto& costFunction = costCollection.get("Simple quadratic cost");
}

TEST_F(StateInputCost_TestFixture, throwsIfCostdoesNotExist) {
  EXPECT_THROW(
      {
        const auto& costFunction = costCollection.get("Nonexisting cost");
        // Should not reach this
      },
      std::out_of_range);
}

TEST_F(StateInputCost_TestFixture, throwsWhenAddExistingCost) {
  std::unique_ptr<bolza::StateInputCost> copy(costCollection.get("Simple quadratic cost").Clone());
  EXPECT_THROW(
      {
        costCollection.Add("Simple quadratic cost", std::move(copy));
        // Should not reach this
      },
      std::runtime_error);
}

TEST_F(StateInputCost_TestFixture, canDeactivateCost) {
  auto& cost1 = costCollection.get<SimpleQuadraticCost>("Simple quadratic cost");
  auto& cost2 = costCollection.get<SimpleQuadraticCost>("Another simple quadratic cost");
  cost1.active_ = false;
  const auto cost = costCollection.GetValue(t, x, u, targetTrajectories, {});
  EXPECT_DOUBLE_EQ(cost, cost2.GetValue(t, x, u, targetTrajectories, {}));
}

TEST_F(StateInputCost_TestFixture, canClone) {
  std::unique_ptr<bolza::StateInputCostCollection> newCollection(costCollection.Clone());
  const auto cost = newCollection->GetValue(t, x, u, targetTrajectories, {});
  EXPECT_NEAR(cost, expectedCost, 1e-6);
}

class SimpleQuadraticFinalCost final : public bolza::StateCost {
 public:
  SimpleQuadraticFinalCost(bolza::matrix_t Q) : Q_(std::move(Q)) {}
  ~SimpleQuadraticFinalCost() override = default;

  SimpleQuadraticFinalCost* Clone() const override { return new SimpleQuadraticFinalCost(*this); }

  bolza::scalar_t GetValue(bolza::scalar_t t, const bolza::vector_t& x, const bolza::TargetTrajectories& targetTrajectories,
                          const bolza::PreComputation&) const override {
    return 0.5 * x.dot(Q_ * x);
  }

  bolza::ScalarFunctionQuadraticApproximation GetQuadraticApproximation(bolza::scalar_t t, const bolza::vector_t& x,
                                                                       const bolza::TargetTrajectories& targetTrajectories,
                                                                       const bolza::PreComputation&) const override {
    bolza::ScalarFunctionQuadraticApproximation quadraticApproximation;
    quadraticApproximation.f = 0.5 * x.dot(Q_ * x);
    quadraticApproximation.dfdx = Q_ * x;
    quadraticApproximation.dfdxx = Q_;
    return quadraticApproximation;
  }

 private:
  bolza::matrix_t Q_;
};

class StateCost_TestFixture : public ::testing::Test {
 public:
  const size_t STATE_DIM = 4;
  const size_t INPUT_DIM = 2;

  StateCost_TestFixture() {
    // Define cost parameters
    bolza::matrix_t Q = bolza::matrix_t::Random(STATE_DIM, STATE_DIM);
    Q = (Q + Q.transpose()).eval();

    x = bolza::vector_t::Random(STATE_DIM);
    t = 0.0;

    auto cost = std::make_unique<SimpleQuadraticFinalCost>(std::move(Q));
    expectedCost = cost->GetValue(t, x, targetTrajectories, {});
    expectedCostApproximation = cost->GetQuadraticApproximation(t, x, targetTrajectories, {});

    costCollection.Add("Simple quadratic final cost", std::move(cost));
  }

  bolza::TargetTrajectories targetTrajectories;
  bolza::StateCostCollection costCollection;

  bolza::vector_t x;
  bolza::scalar_t t;
  bolza::scalar_t expectedCost;
  bolza::ScalarFunctionQuadraticApproximation expectedCostApproximation;
};

TEST_F(StateCost_TestFixture, testStateCost) {
  const auto cost = costCollection.GetValue(t, x, targetTrajectories, {});
  EXPECT_NEAR(cost, expectedCost, 1e-6);
}

TEST_F(StateCost_TestFixture, testStateCostApproximation) {
  const auto cost = costCollection.GetQuadraticApproximation(t, x, targetTrajectories, {});
  EXPECT_NEAR(cost.f, expectedCost, 1e-6);
  EXPECT_TRUE(cost.dfdx.isApprox(expectedCostApproximation.dfdx));
  EXPECT_TRUE(cost.dfdxx.isApprox(expectedCostApproximation.dfdxx));
}
