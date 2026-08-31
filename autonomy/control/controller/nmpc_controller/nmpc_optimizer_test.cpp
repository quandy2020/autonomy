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
 * @file nmpc_optimizer_test.cpp
 * @brief Unit tests for NmpcOptimizer and DifferentialDriveProblem
 */

#include "autonomy/control/controller/nmpc_controller/differential_drive_problem.hpp"
#include "autonomy/control/controller/nmpc_controller/optimizer.hpp"
#include "autonomy/control/controller/nmpc_controller/parameter_defaults.hpp"
#include "autonomy/control/controller/nmpc_controller/path_reference.hpp"

#include "gtest/gtest.h"
#include <automsgs/msgs/nav_msgs/path.pb.h>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {
namespace {

automsgs::msgs::nav_msgs::Path MakeStraightPath() {
    automsgs::msgs::nav_msgs::Path path;
    for (int i = 0; i <= 20; ++i) {
        auto* pose = path.add_poses();
        pose->mutable_pose()->mutable_position()->set_x(static_cast<double>(i) * 0.2);
        pose->mutable_pose()->mutable_position()->set_y(0.0);
        pose->mutable_pose()->mutable_orientation()->set_w(1.0);
    }
    return path;
}

proto::NMPCControllerOptions TestOptions(const std::string& solver_type) {
    auto options = ApplyDefaults(proto::NMPCControllerOptions{});
    options.set_solver_type(solver_type);
    options.set_horizon_steps(15);
    options.set_max_solver_iter(10);
    return options;
}

}  // namespace

class NmpcOptimizerTest : public ::testing::TestWithParam<std::string> {};

TEST_P(NmpcOptimizerTest, SolvesStraightLineTracking) {
    PathReference path;
    path.SetPlan(MakeStraightPath());

    NmpcOptimizer optimizer(TestOptions(GetParam()));
    DifferentialDriveProblem::StateVector state;
    state << 0.0, 0.0, 0.0;

    NmpcOptimizer::SolveResult result;
    ASSERT_TRUE(optimizer.Solve(state, path, 0.0, &result));
    EXPECT_TRUE(std::isfinite(result.cmd(0)));
    EXPECT_TRUE(std::isfinite(result.cmd(1)));
    EXPECT_GT(result.predicted_states.size(), 1u);
}

INSTANTIATE_TEST_SUITE_P(SolverBackends, NmpcOptimizerTest,
                         ::testing::Values("ddp", "fmpc", "cgmres"));

TEST(DifferentialDriveProblemTest, InputInequalityConstraints) {
    auto options = ApplyDefaults(proto::NMPCControllerOptions{});
    DifferentialDriveProblem problem(0.05, options);

    DifferentialDriveProblem::StateVector x =
        DifferentialDriveProblem::StateVector::Zero();
    DifferentialDriveProblem::InputVector u;
    u << options.max_linear_vel() + 0.1, options.max_angular_vel() + 0.1;

    const auto g = problem.ineqConst(0.0, x, u);
    EXPECT_GT(g(0), 0.0);
    EXPECT_GT(g(2), 0.0);
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
