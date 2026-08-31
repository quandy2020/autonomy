/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

/**
 * @file parameter_defaults_test.cpp
 * @brief Unit tests for ApplyDefaults()
 */

#include "autonomy/control/controller/nmpc_controller/parameter_defaults.hpp"

#include "gtest/gtest.h"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

TEST(ParameterDefaultsTest, FillsMissingValues) {
    proto::NMPCControllerOptions options;
    const auto filled = ApplyDefaults(options);

    EXPECT_GT(filled.model_dt(), 0.0);
    EXPECT_GT(filled.horizon_steps(), 0);
    EXPECT_GT(filled.max_linear_vel(), 0.0);
    EXPECT_EQ(filled.solver_type(), "ddp");
    EXPECT_GT(filled.weight_pos(), 0.0);
    EXPECT_GT(filled.collision_cost_threshold(), 0.0);
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
