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

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace planning {
namespace utils {
namespace {

commsgs::geometry_msgs::PoseStamped MakePose(double x, double y) {
    commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    return pose;
}

TEST(PathSimplifierTest, ReturnsUnchangedWhenEpsilonNonPositive) {
    commsgs::planning_msgs::Path path;
    path.poses = {MakePose(0.0, 0.0), MakePose(1.0, 0.0), MakePose(2.0, 0.0)};
    const auto simplified = SimplifyPath(path, 0.0);
    EXPECT_EQ(path.poses.size(), simplified.poses.size());
}

TEST(PathSimplifierTest, ReducesCollinearPoints) {
    commsgs::planning_msgs::Path path;
    path.poses = {MakePose(0.0, 0.0), MakePose(1.0, 0.0), MakePose(2.0, 0.0),
                  MakePose(3.0, 0.0)};
    const auto simplified = SimplifyPath(path, 0.05);
    ASSERT_GE(simplified.poses.size(), 2U);
    EXPECT_LE(simplified.poses.size(), path.poses.size());
    EXPECT_DOUBLE_EQ(simplified.poses.front().pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(simplified.poses.back().pose.position.x, 3.0);
}

TEST(PathSimplifierTest, KeepsCornerPoint) {
    commsgs::planning_msgs::Path path;
    path.poses = {MakePose(0.0, 0.0), MakePose(1.0, 0.0), MakePose(1.0, 1.0),
                  MakePose(1.0, 2.0)};
    const auto simplified = SimplifyPath(path, 0.05);
    ASSERT_EQ(simplified.poses.size(), 3U);
    EXPECT_DOUBLE_EQ(simplified.poses[1].pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(simplified.poses[1].pose.position.y, 0.0);
}

}  // namespace
}  // namespace utils
}  // namespace planning
}  // namespace autonomy
