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

#include "testLoopshapingAugmentedLagrangian.h"

#include "helpers/tools_test.hpp"

using namespace bolza;

TEST(TestFixtureLoopShapingAugmentedLagrangian, testStateInputApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingAugmentedLagrangian test(config);
    test.testStateInputEvaluation();
    test.testStateInputApproximation();
  }
}

TEST(TestFixtureLoopShapingAugmentedLagrangian, testStateApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingAugmentedLagrangian test(config);
    test.testStateEvaluation();
    test.testStateApproximation();
  }
}
