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

#include <bolza/core/penalties/penalties/double_sided_penalty.hpp>
#include <bolza/core/penalties/penalties/squared_hinge_penalty.hpp>

TEST(testDoubleSidedPenalty, value) {
  const bolza::scalar_t eps = 1e-9;
  const bolza::scalar_t t = 0.0;
  const bolza::scalar_t l = -1.0;
  const bolza::scalar_t u = 1.0;
  bolza::SquaredHingePenalty penalty(bolza::SquaredHingePenalty::Config(1.0, 0.0));
  bolza::DoubleSidedPenalty bounds(l, u, std::unique_ptr<bolza::SquaredHingePenalty>(penalty.Clone()));

  EXPECT_NEAR(bounds.GetValue(t, 0.0), 0.0, eps);
  EXPECT_NEAR(bounds.GetValue(t, 2.0), penalty.GetValue(t, u - 2.0), eps);
  EXPECT_NEAR(bounds.GetValue(t, -3.0), penalty.GetValue(t, -3.0 - l), eps);
}

TEST(testDoubleSidedPenalty, derivative) {
  const bolza::scalar_t eps = 1e-9;
  const bolza::scalar_t t = 0.0;
  const bolza::scalar_t l = -1.0;
  const bolza::scalar_t u = 1.0;
  bolza::SquaredHingePenalty penalty(bolza::SquaredHingePenalty::Config(1.0, 0.0));
  bolza::DoubleSidedPenalty bounds(l, u, std::unique_ptr<bolza::SquaredHingePenalty>(penalty.Clone()));

  EXPECT_NEAR(bounds.GetDerivative(t, 0.0), 0.0, eps);
  EXPECT_NEAR(bounds.GetDerivative(t, 2.0), -penalty.GetDerivative(t, u - 2.0), eps);
  EXPECT_NEAR(bounds.GetDerivative(t, -3.0), penalty.GetDerivative(t, -3.0 - l), eps);
}

TEST(testDoubleSidedPenalty, secondDerivative) {
  const bolza::scalar_t eps = 1e-9;
  const bolza::scalar_t t = 0.0;
  const bolza::scalar_t l = -1.0;
  const bolza::scalar_t u = 1.0;
  bolza::SquaredHingePenalty penalty(bolza::SquaredHingePenalty::Config(1.0, 0.0));
  bolza::DoubleSidedPenalty bounds(l, u, std::unique_ptr<bolza::SquaredHingePenalty>(penalty.Clone()));

  EXPECT_NEAR(bounds.GetSecondDerivative(t, 0.0), 0.0, eps);
  EXPECT_NEAR(bounds.GetSecondDerivative(t, 2.0), penalty.GetSecondDerivative(t, u - 2.0), eps);
  EXPECT_NEAR(bounds.GetSecondDerivative(t, -3.0), penalty.GetSecondDerivative(t, -3.0 - l), eps);
}
