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

#include "testLoopshapingConfigurations.h"

using namespace bolza;

class TestFixturePreComputation : LoopshapingTestConfiguration {
 public:
  TestFixturePreComputation(const std::string& configName) : LoopshapingTestConfiguration(configName) {}

  void testCachedStateInput() {
    const auto t = 0.0;
    const auto& x = x_;
    const auto& u = u_;

    const auto x_system = loopshapingDefinition_->GetSystemState(x);
    const auto u_system = loopshapingDefinition_->GetSystemInput(x, u);
    const auto x_filter = loopshapingDefinition_->GetFilterState(x);
    const auto u_filter = loopshapingDefinition_->GetFilteredInput(x, u);

    preComp_->Request(Request::Cost, t, x, u);

    EXPECT_TRUE(x_system.isApprox(preComp_->GetSystemState()));
    EXPECT_TRUE(u_system.isApprox(preComp_->GetSystemInput()));
    EXPECT_TRUE(x_filter.isApprox(preComp_->GetFilterState()));
    EXPECT_TRUE(u_filter.isApprox(preComp_->GetFilteredInput()));
  }
};

TEST(testLoopshapingPreComputation, getCachedStateInput) {
  for (const auto config : configNames) {
    TestFixturePreComputation test(config);
    test.testCachedStateInput();
  }
}
