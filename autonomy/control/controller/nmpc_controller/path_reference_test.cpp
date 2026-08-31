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
 * @file path_reference_test.cpp
 * @brief Unit tests for nmpc_controller::PathReference
 */

#include "autonomy/control/controller/nmpc_controller/path_reference.hpp"

#include <cmath>

#include "gtest/gtest.h"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {
namespace {

automsgs::msgs::nav_msgs::Path MakeStraightPath() {
    automsgs::msgs::nav_msgs::Path path;
    for (int i = 0; i <= 10; ++i) {
        auto* pose = path.add_poses();
        pose->mutable_pose()->mutable_position()->set_x(static_cast<double>(i));
        pose->mutable_pose()->mutable_position()->set_y(0.0);
        pose->mutable_pose()->mutable_orientation()->set_w(1.0);
    }
    return path;
}

}  // namespace

TEST(PathReferenceTest, SamplesAlongStraightPath) {
    PathReference reference;
    reference.SetPlan(MakeStraightPath());

    const auto sample = reference.SampleAlongPath(0.0, 1.0, 3);
    EXPECT_NEAR(sample.x, 3.0, 1e-6);
    EXPECT_NEAR(sample.y, 0.0, 1e-6);
    EXPECT_NEAR(reference.PathLength(), 10.0, 1e-6);
}

TEST(PathReferenceTest, ClosestPointAdvancesMonotonically) {
    PathReference reference;
    reference.SetPlan(MakeStraightPath());

    const auto first =
        reference.ClosestPointOnPath(2.5, 0.2, 0.0, 1.0);
    const auto second =
        reference.ClosestPointOnPath(6.0, 0.0, first.arc_length, 1.0);

    EXPECT_NEAR(first.arc_length, 2.5, 1e-3);
    EXPECT_NEAR(second.arc_length, 6.0, 1e-3);
    EXPECT_LT(first.distance, 0.3);
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
