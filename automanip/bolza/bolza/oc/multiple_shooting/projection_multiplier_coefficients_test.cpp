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

#include <bolza/core/misc/linear_algebra.hpp>
#include <bolza/oc/multiple_shooting/projection_multiplier_coefficients.hpp>

#include "helpers/problems_generation_test.hpp"

using namespace bolza;

TEST(testMultipleShootingHelpers, testProjectionMultiplierCoefficients) {
  constexpr size_t stateDim = 30;
  constexpr size_t inputDim = 20;
  constexpr size_t constraintDim = 10;

  const auto cost = GetRandomCost(stateDim, inputDim);
  const auto dynamics = GetRandomDynamics(stateDim, inputDim);
  const auto constraint = GetRandomConstraints(stateDim, inputDim, constraintDim);

  auto result = LinearAlgebra::QrConstraintProjection(constraint);
  const auto projection = std::move(result.first);
  const auto pseudoInverse = std::move(result.second);

  multiple_shooting::ProjectionMultiplierCoefficients projectionMultiplierCoefficients;
  projectionMultiplierCoefficients.Compute(cost, dynamics, projection, pseudoInverse);

  const matrix_t dfdx = -pseudoInverse * (cost.dfdux + cost.dfduu * projection.dfdx);
  const matrix_t dfdu = -pseudoInverse * (cost.dfduu * projection.dfdu);
  const matrix_t dfdcostate = -pseudoInverse * dynamics.dfdu.transpose();
  const vector_t f = -pseudoInverse * (cost.dfdu + cost.dfduu * projection.f);

  ASSERT_TRUE(projectionMultiplierCoefficients.dfdx.isApprox(dfdx));
  ASSERT_TRUE(projectionMultiplierCoefficients.dfdu.isApprox(dfdu));
  ASSERT_TRUE(projectionMultiplierCoefficients.dfdcostate.isApprox(dfdcostate));
  ASSERT_TRUE(projectionMultiplierCoefficients.f.isApprox(f));
}
