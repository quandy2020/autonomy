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

#include "autonomy/planning/planner_server.hpp"

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "gtest/gtest.h"

namespace autonomy {
namespace planning {
namespace {

proto::PlannerOptions CreateTestPlannerOptions() {
    proto::PlannerOptions options;
    options.add_planner_plugins("navfn_planner");
    options.set_default_planner_id("navfn_planner");
    options.set_costmap_update_timeout(0.5);
    options.mutable_navfn()->set_tolerance(0.1);
    options.mutable_navfn()->set_allow_unknown(true);
    options.mutable_navfn()->set_use_astar(false);

    auto* costmap = options.mutable_costmap();
    costmap->set_enabled(true);
    costmap->set_frame_id("map");
    costmap->set_name("test_costmap");
    costmap->set_resolution(0.05);
    costmap->set_width(10.0);
    costmap->set_height(10.0);
    costmap->add_plugins("none");
    return options;
}

commsgs::geometry_msgs::PoseStamped MakePose(double x, double y) {
    commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    return pose;
}

TEST(PlannerServerTest, ComputesPathOnFreeMap) {
    const auto options = CreateTestPlannerOptions();
    PlannerServer server(options);
    server.Start();

    auto* costmap = server.GetCostmapWrapper()->getCostmap();
    ASSERT_NE(costmap, nullptr);
    costmap->resetMapToValue(0, 0, costmap->getSizeInCellsX(),
                             costmap->getSizeInCellsY(),
                             map::costmap_2d::FREE_SPACE);

    const auto start = MakePose(0.25, 0.25);
    const auto goal = MakePose(0.40, 0.40);
    const auto path = server.ComputePathToPose(start, goal, "navfn_planner",
                                             []() { return false; });

    EXPECT_FALSE(path.poses.empty());
    EXPECT_EQ(1U, server.GetMetrics().plans_succeeded.load());
}

TEST(PlannerServerTest, RestartAfterShutdown) {
    const auto options = CreateTestPlannerOptions();
    PlannerServer server(options);
    server.Start();

    auto* costmap = server.GetCostmapWrapper()->getCostmap();
    ASSERT_NE(costmap, nullptr);
    costmap->resetMapToValue(0, 0, costmap->getSizeInCellsX(),
                             costmap->getSizeInCellsY(),
                             map::costmap_2d::FREE_SPACE);

    server.Shutdown();
    ASSERT_NO_THROW(server.Start());

    const auto path = server.ComputePathToPose(
        MakePose(0.25, 0.25), MakePose(0.40, 0.40), "navfn_planner",
        []() { return false; });
    EXPECT_FALSE(path.poses.empty());
}

TEST(PlannerServerTest, AcceptsEmptyFrameIdAsGlobalFrame) {
    const auto options = CreateTestPlannerOptions();
    PlannerServer server(options);
    server.Start();

    auto* costmap = server.GetCostmapWrapper()->getCostmap();
    ASSERT_NE(costmap, nullptr);
    costmap->resetMapToValue(0, 0, costmap->getSizeInCellsX(),
                             costmap->getSizeInCellsY(),
                             map::costmap_2d::FREE_SPACE);

    auto start = MakePose(0.25, 0.25);
    auto goal = MakePose(0.40, 0.40);
    start.header.frame_id.clear();
    goal.header.frame_id.clear();

    const auto path = server.ComputePathToPose(start, goal, "navfn_planner",
                                             []() { return false; });
    EXPECT_FALSE(path.poses.empty());
    EXPECT_EQ("map", path.header.frame_id);
}

}  // namespace
}  // namespace planning
}  // namespace autonomy
