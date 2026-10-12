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
#include <bolza/core/loopshaping/loopshaping_property_tree.hpp>
#include <bolza/core/loopshaping/dynamics/loopshaping_filter_dynamics.hpp>
#include "testLoopshapingConfigurations.h"

using namespace bolza;

TEST(testLoopshapingFilterDynamics, Integration) {
  constexpr size_t FILTER_STATE_DIM = 3;
  constexpr size_t INPUT_DIM = 1;
  matrix_t A(FILTER_STATE_DIM, FILTER_STATE_DIM), B(FILTER_STATE_DIM, INPUT_DIM), C(INPUT_DIM, FILTER_STATE_DIM), D(INPUT_DIM, INPUT_DIM);
  A << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
  B << 1.0, 1.0, 1.0;
  C << 0.0, 0.0, 0.0;
  D << 1.0;
  Filter filter(A, B, C, D);
  auto loopshapingDefinition = std::make_shared<LoopshapingDefinition>(LoopshapingType::outputpattern, filter);
  using TestLoopshapingFilterDynamics = LoopshapingFilterDynamics;
  TestLoopshapingFilterDynamics loopshapingFilterDynamics(loopshapingDefinition);

  // Initial conditions
  vector_t input, filter_state, filter_state0;
  input.setZero(INPUT_DIM);
  filter_state0.setOnes(FILTER_STATE_DIM);

  // Integrate
  scalar_t dt = 1.0;
  loopshapingFilterDynamics.SetFilterState(filter_state0);
  loopshapingFilterDynamics.Integrate(dt, input);
  filter_state = loopshapingFilterDynamics.GetFilterState();

  for (int d = 0; d < FILTER_STATE_DIM; ++d) {
    // Check against analytical solution assuming diagonal A and zero input
    ASSERT_NEAR(filter_state(d), filter_state0(d) * exp(A(d, d) * dt), 1e-3);
  }
}
