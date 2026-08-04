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

#include "autonomy/planning/planner/dijkstra/dijkstra_planner.hpp"
#include "autonomy/planning/planner/navfn/navfn_planner.hpp"
#include "autonomy/planning/planner/theta_star/theta_star_planner.hpp"

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "gtest/gtest.h"

namespace autonomy {
namespace planning {
namespace {

map::costmap_2d::Costmap2DWrapper::SharedPtr CreateTestCostmapWrapper() {
    map::proto::Costmap2DOptions options;
    options.set_enabled(true);
    options.set_frame_id("map");
    options.set_name("test_costmap");
    options.set_resolution(0.05);
    options.set_width(10.0);
    options.set_height(10.0);
    options.add_plugins("none");
    return std::make_shared<map::costmap_2d::Costmap2DWrapper>(options,
                                                               "test_costmap");
}

proto::PlannerOptions CreatePlannerOptions() {
    proto::PlannerOptions options;
    options.mutable_navfn()->set_tolerance(0.1);
    options.mutable_navfn()->set_use_astar(false);
    options.mutable_navfn()->set_allow_unknown(true);
    options.mutable_navfn()->set_use_final_approach_orientation(false);
    return options;
}

automsgs::msgs::geometry_msgs::PoseStamped MakePose(double x, double y) {
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_header()->set_frame_id("map");
    pose.mutable_pose()->mutable_position()->set_x(x);
    pose.mutable_pose()->mutable_position()->set_y(y);
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    return pose;
}

TEST(NavfnPlannerTest, DoesNotMutateGlobalCostmapWhenClearingStartCell) {
    auto costmap_wrapper = CreateTestCostmapWrapper();
    auto* costmap = costmap_wrapper->getCostmap();
    ASSERT_NE(costmap, nullptr);

    const unsigned int start_mx = 5;
    const unsigned int start_my = 5;
    costmap->resetMapToValue(0, 0, costmap->getSizeInCellsX(),
                             costmap->getSizeInCellsY(),
                             map::costmap_2d::FREE_SPACE);
    costmap->setCost(start_mx, start_my,
                     map::costmap_2d::LETHAL_OBSTACLE);
    const unsigned char cost_before = costmap->getCost(start_mx, start_my);

    planner::navfn::NavfnPlanner planner(CreatePlannerOptions(), "navfn_planner",
                                       costmap_wrapper);

    automsgs::msgs::nav_msgs::Path path;
    const auto start = MakePose(start_mx * 0.05, start_my * 0.05);
    const auto goal = MakePose(0.40, 0.40);
    const uint32_t code =
        planner.CreatePlan(start, goal, path, []() { return false; });

    EXPECT_EQ(cost_before, costmap->getCost(start_mx, start_my));
    EXPECT_EQ(static_cast<uint32_t>(proto::PlannerResultCode::PLANNER_SUCCESS),
              code);
    EXPECT_FALSE(path.poses().empty());
}

TEST(NavfnPlannerTest, ReturnsCanceledWhenCancelled) {
    auto costmap_wrapper = CreateTestCostmapWrapper();
    auto* costmap = costmap_wrapper->getCostmap();
    ASSERT_NE(costmap, nullptr);
    costmap->resetMapToValue(0, 0, costmap->getSizeInCellsX(),
                             costmap->getSizeInCellsY(),
                             map::costmap_2d::FREE_SPACE);

    planner::navfn::NavfnPlanner planner(CreatePlannerOptions(), "navfn_planner",
                                       costmap_wrapper);

    automsgs::msgs::nav_msgs::Path path;
    const auto start = MakePose(0.25, 0.25);
    const auto goal = MakePose(0.40, 0.40);
    const uint32_t code =
        planner.CreatePlan(start, goal, path, []() { return true; });

    EXPECT_EQ(static_cast<uint32_t>(proto::PlannerResultCode::PLANNER_CANCELED),
              code);
}

TEST(DijkstraPlannerTest, ForcesDijkstraSearch) {
    auto costmap_wrapper = CreateTestCostmapWrapper();
    auto* costmap = costmap_wrapper->getCostmap();
    ASSERT_NE(costmap, nullptr);
    costmap->resetMapToValue(0, 0, costmap->getSizeInCellsX(),
                             costmap->getSizeInCellsY(),
                             map::costmap_2d::FREE_SPACE);

    proto::PlannerOptions options;
    options.mutable_dijkstra()->set_tolerance(0.15);
    options.mutable_dijkstra()->set_allow_unknown(true);
    options.mutable_dijkstra()->set_use_final_approach_orientation(true);

    planner::dijkstra::DijkstraPlanner planner(options, "dijkstra_planner",
                                             costmap_wrapper);

    automsgs::msgs::nav_msgs::Path path;
    const auto start = MakePose(0.25, 0.25);
    const auto goal = MakePose(0.40, 0.40);
    const uint32_t code =
        planner.CreatePlan(start, goal, path, []() { return false; });

    EXPECT_EQ(static_cast<uint32_t>(proto::PlannerResultCode::PLANNER_SUCCESS),
              code);
    EXPECT_FALSE(path.poses().empty());
}

TEST(ThetaStarPlannerTest, FindsPathOnFreeMap) {
    auto costmap_wrapper = CreateTestCostmapWrapper();
    auto* costmap = costmap_wrapper->getCostmap();
    ASSERT_NE(costmap, nullptr);
    costmap->resetMapToValue(0, 0, costmap->getSizeInCellsX(),
                             costmap->getSizeInCellsY(),
                             map::costmap_2d::FREE_SPACE);

    proto::PlannerOptions options;
    options.mutable_theta_star()->set_how_many_corners(8);
    options.mutable_theta_star()->set_allow_unknown(true);
    options.mutable_theta_star()->set_w_euc_cost(2.0);
    options.mutable_theta_star()->set_w_traversal_cost(1.0);
    options.mutable_theta_star()->set_w_heuristic_cost(1.0);

    planner::theta_star::ThetaStarPlanner planner(options, "theta_star_planner",
                                                costmap_wrapper);

    automsgs::msgs::nav_msgs::Path path;
    const auto start = MakePose(0.25, 0.25);
    const auto goal = MakePose(0.40, 0.40);
    const uint32_t code =
        planner.CreatePlan(start, goal, path, []() { return false; });

    EXPECT_EQ(static_cast<uint32_t>(proto::PlannerResultCode::PLANNER_SUCCESS),
              code);
    EXPECT_FALSE(path.poses().empty());
}

}  // namespace
}  // namespace planning
}  // namespace autonomy
