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

#include "autonomy/planning/utils/path_simplifier.hpp"

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include "gtest/gtest.h"

namespace autonomy {
namespace planning {
namespace utils {
namespace {

automsgs::msgs::geometry_msgs::PoseStamped MakePose(double x, double y) {
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_header()->set_frame_id("map");
    pose.mutable_pose()->mutable_position()->set_x(x);
    pose.mutable_pose()->mutable_position()->set_y(y);
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    return pose;
}

TEST(PathSimplifierTest, ReturnsUnchangedWhenEpsilonNonPositive) {
    automsgs::msgs::nav_msgs::Path path;
    *path.add_poses() = MakePose(0.0, 0.0);
    *path.add_poses() = MakePose(1.0, 0.0);
    *path.add_poses() = MakePose(2.0, 0.0);
    const auto simplified = SimplifyPath(path, 0.0);
    EXPECT_EQ(path.poses_size(), simplified.poses_size());
}

TEST(PathSimplifierTest, ReducesCollinearPoints) {
    automsgs::msgs::nav_msgs::Path path;
    *path.add_poses() = MakePose(0.0, 0.0);
    *path.add_poses() = MakePose(1.0, 0.0);
    *path.add_poses() = MakePose(2.0, 0.0);
    *path.add_poses() = MakePose(3.0, 0.0);
    const auto simplified = SimplifyPath(path, 0.05);
    ASSERT_GE(simplified.poses_size(), 2U);
    EXPECT_LE(simplified.poses_size(), path.poses_size());
    EXPECT_DOUBLE_EQ(simplified.poses(0).pose().position().x(), 0.0);
    EXPECT_DOUBLE_EQ(simplified.poses(simplified.poses_size() - 1).pose().position().x(), 3.0);
}

TEST(PathSimplifierTest, KeepsCornerPoint) {
    automsgs::msgs::nav_msgs::Path path;
    *path.add_poses() = MakePose(0.0, 0.0);
    *path.add_poses() = MakePose(1.0, 0.0);
    *path.add_poses() = MakePose(1.0, 1.0);
    *path.add_poses() = MakePose(1.0, 2.0);
    const auto simplified = SimplifyPath(path, 0.05);
    ASSERT_EQ(simplified.poses_size(), 3U);
    EXPECT_DOUBLE_EQ(simplified.poses(1).pose().position().x(), 1.0);
    EXPECT_DOUBLE_EQ(simplified.poses(1).pose().position().y(), 0.0);
}

}  // namespace
}  // namespace utils
}  // namespace planning
}  // namespace autonomy
