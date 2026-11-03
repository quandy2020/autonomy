/*
 * Copyright 2026 The Openbot Authors
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

#include "atlas/match/ml_pnp_solver.hpp"

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::internal {
namespace {

int CountIterations(int total_iterations, int maximum_iterations,
                    int requested_batch_iterations) {
    int batch_iterations = 0;
    while (ShouldContinueRansacIterations(
        total_iterations, maximum_iterations, batch_iterations,
        requested_batch_iterations)) {
        ++total_iterations;
        ++batch_iterations;
    }
    return batch_iterations;
}

TEST(MLPnPSolverIterationPolicyTest, StopsAtRequestedBatchSize) {
    EXPECT_EQ(CountIterations(0, 300, 5), 5);
    EXPECT_EQ(CountIterations(42, 300, 5), 5);
}

TEST(MLPnPSolverIterationPolicyTest, StopsAtRemainingGlobalBudget) {
    EXPECT_EQ(CountIterations(298, 300, 5), 2);
    EXPECT_EQ(CountIterations(300, 300, 5), 0);
}

TEST(MLPnPSolverIterationPolicyTest, EmptyBatchDoesNotIterate) {
    EXPECT_EQ(CountIterations(0, 300, 0), 0);
}

}  // namespace
}  // namespace autonomy::localization::atlas::internal
