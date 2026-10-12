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

#include <iostream>

#include <gtest/gtest.h>

#include <bolza/core/initialization/operating_points.hpp>

class InitializationTest : public testing::Test {
 protected:
  static constexpr size_t stateDim_ = 3;
  static constexpr size_t inputDim_ = 2;

  using OperatingPoints = bolza::OperatingPoints;
  using scalar_t = bolza::scalar_t;
  using scalar_array_t = bolza::scalar_array_t;
  using vector_t = bolza::vector_t;
  using vector_array_t = bolza::vector_array_t;

  InitializationTest() = default;

  vector_t input, nextState;
};

TEST_F(InitializationTest, SingleOperatingPoint) {
  const scalar_t t0 = 0.0;
  const scalar_t tf = 1.0;
  const vector_array_t xTraj = {vector_t::Random(stateDim_)};
  const vector_array_t uTraj = {vector_t::Random(inputDim_)};
  OperatingPoints operatingPoints({t0}, xTraj, uTraj);
  operatingPoints.Compute(t0, xTraj[0], tf, input, nextState);

  ASSERT_TRUE(nextState.isApprox(xTraj[0]));
  ASSERT_TRUE(input.isApprox(uTraj[0]));
}

TEST_F(InitializationTest, ZeroTimeInterval) {
  const scalar_t t0 = 0.0;
  const vector_array_t xTraj{vector_t::Random(stateDim_)};
  const vector_array_t uTraj = {vector_t::Random(inputDim_)};
  OperatingPoints operatingPoints({t0}, xTraj, uTraj);
  operatingPoints.Compute(t0, xTraj[0], t0, input, nextState);

  ASSERT_TRUE(nextState.isApprox(xTraj[0]));
  ASSERT_TRUE(input.isApprox(uTraj[0]));
}

TEST_F(InitializationTest, Trajectory) {
  constexpr size_t N = 20;
  scalar_array_t tTraj(N);
  scalar_t n = 0;
  std::generate(tTraj.begin(), tTraj.end(), [&n]() mutable { return n++; });
  vector_array_t xTraj(N);
  std::generate(xTraj.begin(), xTraj.end(), [&]() { return vector_t::Random(stateDim_); });
  vector_array_t uTraj(N);
  std::generate(uTraj.begin(), uTraj.end(), [&]() { return vector_t::Random(inputDim_); });
  OperatingPoints operatingPoints(tTraj, xTraj, uTraj);

  for (size_t i = 0; i < N - 1; i++) {
    operatingPoints.Compute(tTraj[i], xTraj[i], tTraj[i + 1], input, nextState);
    ASSERT_TRUE(input.isApprox(uTraj[i]));
    ASSERT_TRUE(nextState.isApprox(xTraj[i + 1]));
  }
}
