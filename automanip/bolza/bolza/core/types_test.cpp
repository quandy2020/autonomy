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

#include <bolza/core/types.hpp>

namespace {
// Tests if the convention for state only quadratic approximation works with the defined operations (addition and scalar multiplication)

void stateOnlyLinearOperationsTest(const bolza::ScalarFunctionLinearApproximation& value, size_t nx) {
  // Define state only 'other' value
  const bolza::ScalarFunctionLinearApproximation other = [=] {
    bolza::ScalarFunctionLinearApproximation other;
    other.f = 1.5;
    other.dfdx.setConstant(nx, 2.3);
    return other;
  }();

  {  // test value + other
    auto result = value;
    result += other;
    EXPECT_DOUBLE_EQ(result.f, value.f + other.f);
    EXPECT_TRUE(result.dfdx.isApprox(value.dfdx + other.dfdx));
  }

  {  // test other + value
    auto result = other;
    result += value;
    EXPECT_DOUBLE_EQ(result.f, value.f + other.f);
    EXPECT_TRUE(result.dfdx.isApprox(value.dfdx + other.dfdx));
  }

  {  // test  value *= scalar
    const bolza::scalar_t s{2.0};
    auto result = value;
    result *= s;
    EXPECT_DOUBLE_EQ(result.f, s * value.f);
    EXPECT_TRUE(result.dfdx.isApprox(s * value.dfdx));
  }
}

void stateOnlyQuadraticOperationsTest(const bolza::ScalarFunctionQuadraticApproximation& value, size_t nx) {
  // Define state only 'other' value
  const bolza::ScalarFunctionQuadraticApproximation other = [=] {
    bolza::ScalarFunctionQuadraticApproximation other;
    other.f = 1.5;
    other.dfdx.setConstant(nx, 2.3);
    other.dfdxx.setConstant(nx, nx, 8.6);
    return other;
  }();

  {  // test value + other
    auto result = value;
    result += other;
    EXPECT_DOUBLE_EQ(result.f, value.f + other.f);
    EXPECT_TRUE(result.dfdx.isApprox(value.dfdx + other.dfdx));
    EXPECT_TRUE(result.dfdxx.isApprox(value.dfdxx + other.dfdxx));
  }

  {  // test other + value
    auto result = other;
    result += value;
    EXPECT_DOUBLE_EQ(result.f, value.f + other.f);
    EXPECT_TRUE(result.dfdx.isApprox(value.dfdx + other.dfdx));
    EXPECT_TRUE(result.dfdxx.isApprox(value.dfdxx + other.dfdxx));
  }

  {  // test  value *= scalar
    const bolza::scalar_t s{2.0};
    auto result = value;
    result *= s;
    EXPECT_DOUBLE_EQ(result.f, s * value.f);
    EXPECT_TRUE(result.dfdx.isApprox(s * value.dfdx));
    EXPECT_TRUE(result.dfdxx.isApprox(s * value.dfdxx));
  }
}

}  // namespace

TEST(testTypes, stateOnlyLinearOperations_constructor) {
  const int nx = 2;
  auto constructed = bolza::ScalarFunctionLinearApproximation(nx);
  // Need to set values after resize.
  constructed.f = 0.5;
  constructed.dfdx.setConstant(1.0);
  stateOnlyLinearOperationsTest(constructed, nx);
}

TEST(testTypes, stateOnlyLinearOperations_zero) {
  const int nx = 2;
  stateOnlyLinearOperationsTest(bolza::ScalarFunctionLinearApproximation::Zero(nx), nx);
}

TEST(testTypes, stateOnlyLinearOperations_setZero) {
  const int nx = 2;
  bolza::ScalarFunctionLinearApproximation zero;
  zero.setZero(nx);
  stateOnlyLinearOperationsTest(zero, nx);
}

TEST(testTypes, stateOnlyLinearOperations_resize) {
  const int nx = 2;
  bolza::ScalarFunctionLinearApproximation resized(nx + 1, 10);
  resized.resize(nx);
  // Need to set values after resize.
  resized.f = 0.5;
  resized.dfdx.setConstant(1.0);
  stateOnlyLinearOperationsTest(resized, nx);
}

TEST(testTypes, stateOnlyQuadraticOperations_constructor) {
  const int nx = 2;
  auto constructed = bolza::ScalarFunctionQuadraticApproximation(nx);
  // Need to set values after resize.
  constructed.f = 0.5;
  constructed.dfdx.setConstant(1.0);
  constructed.dfdxx.setConstant(2.0);
  stateOnlyQuadraticOperationsTest(constructed, nx);
}

TEST(testTypes, stateOnlyQuadraticOperations_zero) {
  const int nx = 2;
  stateOnlyQuadraticOperationsTest(bolza::ScalarFunctionQuadraticApproximation::Zero(nx), nx);
}

TEST(testTypes, stateOnlyQuadraticOperations_setZero) {
  const int nx = 2;
  bolza::ScalarFunctionQuadraticApproximation zero;
  zero.setZero(nx);
  stateOnlyQuadraticOperationsTest(zero, nx);
}

TEST(testTypes, stateOnlyQuadraticOperations_resize) {
  const int nx = 2;
  bolza::ScalarFunctionQuadraticApproximation resized(nx + 1, 10);
  resized.resize(nx);
  // Need to set values after resize.
  resized.f = 0.5;
  resized.dfdx.setConstant(1.0);
  resized.dfdxx.setConstant(2.0);
  stateOnlyQuadraticOperationsTest(resized, nx);
}
