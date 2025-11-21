/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "autonomy/common/state_estimation/noise_model/uniform_noise.hpp"
#include "autonomy/common/state_vector/generic_state.hpp"

#include "gtest/gtest.h"

#include <memory>
#include <vector>

namespace autonomy {
namespace common {
namespace state_estimation {

using autonomy::common::types::float32_t;
using autonomy::common::state_vector::FloatState;
using autonomy::common::state_vector::variable::X;
using autonomy::common::state_vector::variable::Y;
using autonomy::common::state_estimation::UniformNoise;

namespace
{
float32_t squared(float32_t num)
{
  return num * num;
}

using StateXY = FloatState<X, Y>;

constexpr float32_t kSigmaX = 42.42F;
constexpr float32_t kSigmaY = 23.23F;

}  // namespace


class CreationTest : public ::testing::TestWithParam<UniformNoise<StateXY>> {};

/// @test Test that the noise model is correctly created.
TEST_P(CreationTest, CreateFromVariances) {
  const auto & noise = GetParam();
  const auto dt = std::chrono::milliseconds{100LL};
  const auto covariance = noise.covariance(dt);
  const auto t = std::chrono::duration<float32_t>{dt}.count();

  EXPECT_FLOAT_EQ(covariance(0, 0), squared(kSigmaX) * t);
  EXPECT_FLOAT_EQ(covariance(1, 1), squared(kSigmaY) * t);
  EXPECT_FLOAT_EQ(covariance(1, 0), 0.0F);
  EXPECT_FLOAT_EQ(covariance(0, 1), 0.0F);
}

/// @test Test that the noise model is correctly created.
TEST(UniformNoiseTest, FailWhenWrongNumberOfVariancesPassed) {
  EXPECT_THROW((UniformNoise<StateXY>{std::vector<float32_t>{1.F, 2.F, 3.F}}), std::runtime_error);
  EXPECT_THROW((UniformNoise<StateXY>{std::vector<float32_t>{1.0F}}), std::runtime_error);
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autonomy
