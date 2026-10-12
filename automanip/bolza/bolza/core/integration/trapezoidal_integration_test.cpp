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

#include <algorithm>

#include <bolza/core/types.hpp>
#include <bolza/core/integration/trapezoidal_integration.hpp>

TEST(TrapezoidalIntegrationTest, Rectangle) {
  const bolza::scalar_t width = 8.0;
  const bolza::scalar_t height = 2.0;
  const size_t numIntervals = 100;
  const bolza::scalar_t dx = width / numIntervals;

  bolza::scalar_array_t xTrj(numIntervals + 1, 0.0);
  auto x = -dx;
  std::generate(xTrj.begin(), xTrj.end(), [&x, dx]() { return (x += dx); });

  bolza::scalar_array_t yTrj(numIntervals + 1, height);

  const auto area = bolza::TrapezoidalIntegration(xTrj, yTrj, 0.0);
  ASSERT_NEAR(area, width * height, 1e-12);
}

TEST(TrapezoidalIntegrationTest, RightTriangle) {
  const bolza::scalar_t width = 8.0;
  const bolza::scalar_t height = 2.0;
  const size_t numIntervals = 10;
  const bolza::scalar_t dx = width / numIntervals;
  const bolza::scalar_t dy = height / numIntervals;

  bolza::scalar_array_t xTrj(numIntervals + 1, 0.0);
  auto x = -dx;
  std::generate(xTrj.begin(), xTrj.end(), [&x, dx]() { return (x += dx); });

  bolza::scalar_array_t yTrj(numIntervals + 1, 0.0);
  auto y = -dy;
  std::generate(yTrj.begin(), yTrj.end(), [&y, dy]() { return (y += dy); });

  const auto area = bolza::TrapezoidalIntegration(xTrj, yTrj, 0.0);
  ASSERT_NEAR(area, width * height / 2.0, 1e-12);
}
