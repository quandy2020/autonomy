/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file planner_test.cpp
 * @brief Safety and ranking contract tests for the Shadow path selector.
 */

#include "autonomy/perception/shadow/planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

using Odometry = automsgs::msgs::nav_msgs::Odometry;
using Path = automsgs::msgs::nav_msgs::Path;
using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;

constexpr double kPi = 3.14159265358979323846;

struct Waypoint {
    double x;
    double y;
    double yaw;
};

proto::ShadowOptions Options() {
    proto::ShadowOptions options;
    options.set_map_frame("map");
    options.set_base_frame("base_link");
    options.set_robot_radius(0.20F);
    options.set_inflation_radius(0.40F);
    options.set_max_linear_speed(1.0F);
    options.set_max_angular_speed(1.0F);
    options.set_follow_distance(1.0F);
    options.set_learned_weight(1.0F);
    options.set_clearance_weight(1.0F);
    options.set_traversability_weight(1.0F);
    options.set_curvature_weight(1.0F);
    options.set_progress_weight(1.0F);
    options.set_distance_weight(1.0F);
    options.set_visibility_weight(1.0F);
    return options;
}

void UseOnlyWeight(const char* name, proto::ShadowOptions* options) {
    options->set_learned_weight(0.0F);
    options->set_clearance_weight(0.0F);
    options->set_traversability_weight(0.0F);
    options->set_curvature_weight(0.0F);
    options->set_progress_weight(0.0F);
    options->set_distance_weight(0.0F);
    options->set_visibility_weight(0.0F);
    if (std::string(name) == "learned") {
        options->set_learned_weight(1.0F);
    } else if (std::string(name) == "clearance") {
        options->set_clearance_weight(1.0F);
    } else if (std::string(name) == "traversability") {
        options->set_traversability_weight(1.0F);
    } else if (std::string(name) == "curvature") {
        options->set_curvature_weight(1.0F);
    } else if (std::string(name) == "progress") {
        options->set_progress_weight(1.0F);
    } else if (std::string(name) == "distance") {
        options->set_distance_weight(1.0F);
    } else if (std::string(name) == "visibility") {
        options->set_visibility_weight(1.0F);
    }
}

grid_map::GridMap OpenGrid(double length = 6.0, double resolution = 0.10,
                           double center_x = 0.0, double center_y = 0.0) {
    grid_map::GridMap grid(
        {"elevation", "variance", "obstacle", "traversability"});
    grid.setGeometry(grid_map::Length(length, length), resolution,
                     grid_map::Position(center_x, center_y));
    grid.setFrameId("map");
    grid.get("elevation").setConstant(0.0F);
    grid.get("variance").setConstant(0.0F);
    grid.get("obstacle").setConstant(0.0F);
    grid.get("traversability").setConstant(0.0F);
    return grid;
}

void SetCell(grid_map::GridMap* grid, double x, double y, float elevation,
             float variance, float obstacle, float traversability) {
    const grid_map::Position position(x, y);
    grid->atPosition("elevation", position) = elevation;
    grid->atPosition("variance", position) = variance;
    grid->atPosition("obstacle", position) = obstacle;
    grid->atPosition("traversability", position) = traversability;
}

void SetTraversabilityBand(grid_map::GridMap* grid, double y, float value) {
    for (double x = 0.0; x <= 0.8; x += grid->getResolution()) {
        for (double offset = -0.2; offset <= 0.2;
             offset += grid->getResolution()) {
            grid->atPosition("traversability",
                             grid_map::Position(x, y + offset)) = value;
        }
    }
}

Odometry Odom(double x = 0.0, double y = 0.0, double yaw = 0.0) {
    Odometry odometry;
    odometry.mutable_header()->set_frame_id("map");
    odometry.set_child_frame_id("base_link");
    auto* pose = odometry.mutable_pose()->mutable_pose()->mutable_pose();
    pose->mutable_position()->set_x(x);
    pose->mutable_position()->set_y(y);
    pose->mutable_position()->set_z(0.0);
    pose->mutable_orientation()->set_z(std::sin(yaw * 0.5));
    pose->mutable_orientation()->set_w(std::cos(yaw * 0.5));
    return odometry;
}

PoseStamped Target(double x = 2.0, double y = 0.0) {
    PoseStamped target;
    target.mutable_header()->set_frame_id("map");
    target.mutable_pose()->mutable_position()->set_x(x);
    target.mutable_pose()->mutable_position()->set_y(y);
    target.mutable_pose()->mutable_position()->set_z(0.0);
    target.mutable_pose()->mutable_orientation()->set_w(1.0);
    return target;
}

Path Candidate(std::initializer_list<Waypoint> waypoints) {
    Path path;
    path.mutable_header()->set_frame_id("base_link");
    for (const Waypoint& waypoint : waypoints) {
        auto* pose = path.add_poses();
        pose->mutable_header()->set_frame_id("base_link");
        pose->mutable_pose()->mutable_position()->set_x(waypoint.x);
        pose->mutable_pose()->mutable_position()->set_y(waypoint.y);
        pose->mutable_pose()->mutable_position()->set_z(0.0);
        pose->mutable_pose()->mutable_orientation()->set_z(
            std::sin(waypoint.yaw * 0.5));
        pose->mutable_pose()->mutable_orientation()->set_w(
            std::cos(waypoint.yaw * 0.5));
    }
    return path;
}

void SeedOutput(Path* output) {
    output->mutable_header()->set_frame_id("stale");
    output->add_poses()->mutable_header()->set_frame_id("stale");
}

TEST(FollowPlannerTest, TransformsToMapAndPrependsCurrentRobotPose) {
    const FollowPlanner planner(Options());
    Path output;

    ASSERT_TRUE(planner.Select(
        2'000'000'007, {Candidate({{0.25, 0.0, 0.0}, {0.50, 0.0, kPi / 4.0}})},
        {0.1F}, OpenGrid(6.0, 0.10, 1.0, 2.0), Odom(1.0, 2.0, kPi / 2.0),
        Target(2.0, 2.0), &output));

    EXPECT_EQ(output.header().frame_id(), "map");
    EXPECT_EQ(output.header().stamp().sec(), 2);
    EXPECT_EQ(output.header().stamp().nanosec(), 7U);
    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_DOUBLE_EQ(output.poses(0).pose().position().x(), 1.0);
    EXPECT_DOUBLE_EQ(output.poses(0).pose().position().y(), 2.0);
    EXPECT_NEAR(output.poses(1).pose().position().x(), 1.0, 1.0e-9);
    EXPECT_NEAR(output.poses(1).pose().position().y(), 2.25, 1.0e-9);
    EXPECT_NEAR(output.poses(2).pose().position().x(), 1.0, 1.0e-9);
    EXPECT_NEAR(output.poses(2).pose().position().y(), 2.50, 1.0e-9);
    for (const auto& pose : output.poses()) {
        EXPECT_EQ(pose.header().frame_id(), "map");
        EXPECT_EQ(pose.header().stamp().sec(), 2);
        EXPECT_EQ(pose.header().stamp().nanosec(), 7U);
    }
}

TEST(FollowPlannerTest, RejectsAPathWhoseCircularFootprintLeavesTheMap) {
    auto options = Options();
    options.set_robot_radius(0.20F);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(planner.Select(2'000'000'000,
                               {Candidate({{0.4, 0.0, 0.0}, {0.8, 0.0, 0.0}}),
                                Candidate({{0.0, 0.3, 0.0}, {0.0, 0.6, 0.0}})},
                               {0.0F, 1.0F}, OpenGrid(2.0), Odom(), Target(),
                               &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(2).pose().position().x(), 0.0, 1.0e-9);
    EXPECT_NEAR(output.poses(2).pose().position().y(), 0.6, 1.0e-9);
}

TEST(FollowPlannerTest, SamplesSegmentsAtHalfCellAndRejectsUnknownCells) {
    auto grid = OpenGrid();
    const float unknown = std::numeric_limits<float>::quiet_NaN();
    SetCell(&grid, 0.5, 0.0, unknown, unknown, unknown, unknown);
    const FollowPlanner planner(Options());
    Path output;

    ASSERT_TRUE(
        planner.Select(2'000'000'000,
                       {Candidate({{0.2, 0.0, 0.0}, {0.8, 0.0, 0.0}}),
                        Candidate({{0.0, -0.3, 0.0}, {0.0, -0.6, 0.0}})},
                       {0.0F, 1.0F}, grid, Odom(), Target(), &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(2).pose().position().x(), 0.0, 1.0e-9);
    EXPECT_NEAR(output.poses(2).pose().position().y(), -0.6, 1.0e-9);
}

TEST(FollowPlannerTest, RejectsObstacleIntersectingOnlyTheRobotFootprint) {
    auto options = Options();
    options.set_robot_radius(0.20F);
    auto grid = OpenGrid();
    SetCell(&grid, 0.5, 0.25, 0.0F, 0.0F, 0.5F, 0.0F);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(
        planner.Select(2'000'000'000,
                       {Candidate({{0.25, 0.0, 0.0}, {0.50, 0.0, 0.0}}),
                        Candidate({{0.25, -0.4, 0.0}, {0.50, -0.4, 0.0}})},
                       {0.0F, 1.0F}, grid, Odom(), Target(), &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(2).pose().position().y(), -0.4, 1.0e-9);
}

TEST(FollowPlannerTest, RejectsObstacleAndTraversabilityThresholds) {
    const FollowPlanner planner(Options());
    for (const auto& blocked_values :
         std::vector<std::pair<float, float>>{{0.5F, 0.0F}, {0.0F, 1.0F}}) {
        auto grid = OpenGrid();
        SetCell(&grid, 0.4, 0.0, 0.0F, 0.0F, blocked_values.first,
                blocked_values.second);
        Path output;

        ASSERT_TRUE(planner.Select(2'000'000'000,
                                   {Candidate({{0.4, 0.0, 0.0}})}, {0.1F}, grid,
                                   Odom(), Target(), &output));
        EXPECT_TRUE(output.poses().empty());
    }
}

TEST(FollowPlannerTest, RejectsPathExceedingLinearSpeedLimit) {
    auto options = Options();
    options.set_max_linear_speed(0.5F);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(planner.Select(
        2'000'000'000,
        {Candidate({{0.75, 0.0, 0.0}}), Candidate({{0.4, 0.0, 0.0}})},
        {0.0F, 1.0F}, OpenGrid(), Odom(), Target(), &output));

    ASSERT_EQ(output.poses_size(), 2);
    EXPECT_NEAR(output.poses(1).pose().position().x(), 0.4, 1.0e-9);
}

TEST(FollowPlannerTest, RejectsPathExceedingAngularSpeedLimit) {
    auto options = Options();
    options.set_max_angular_speed(0.5F);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(planner.Select(
        2'000'000'000,
        {Candidate({{0.4, 0.0, 0.75}}), Candidate({{0.4, 0.0, 0.25}})},
        {0.0F, 1.0F}, OpenGrid(), Odom(), Target(), &output));

    ASSERT_EQ(output.poses_size(), 2);
    EXPECT_NEAR(output.poses(1).pose().orientation().z(), std::sin(0.125),
                1.0e-9);
}

TEST(FollowPlannerTest, PrefersTerminalFollowDistance) {
    auto options = Options();
    UseOnlyWeight("distance", &options);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(planner.Select(
        2'000'000'000,
        {Candidate({{0.5, 0.0, 0.0}, {1.0, 0.0, 0.0}}),
         Candidate({{0.25, 0.0, 0.0}, {0.50, 0.0, 0.0}})},
        {0.5F, 0.5F}, OpenGrid(), Odom(), Target(2.0, 0.0), &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(2).pose().position().x(), 1.0, 1.0e-9);
}

TEST(FollowPlannerTest, PrefersGreaterObstacleClearance) {
    auto options = Options();
    options.set_robot_radius(0.15F);
    UseOnlyWeight("clearance", &options);
    auto grid = OpenGrid();
    SetCell(&grid, 0.6, 0.35, 0.0F, 0.0F, 1.0F, 0.0F);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(
        planner.Select(2'000'000'000,
                       {Candidate({{0.3, 0.0, 0.0}, {0.6, 0.0, 0.0}}),
                        Candidate({{0.3, -0.6, 0.0}, {0.6, -0.6, 0.0}})},
                       {0.5F, 0.5F}, grid, Odom(), Target(), &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(2).pose().position().y(), -0.6, 1.0e-9);
}

TEST(FollowPlannerTest, PrefersLowerMeanTraversability) {
    auto options = Options();
    UseOnlyWeight("traversability", &options);
    auto grid = OpenGrid();
    SetTraversabilityBand(&grid, 0.5, 0.8F);
    SetTraversabilityBand(&grid, -0.5, 0.1F);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(
        planner.Select(2'000'000'000,
                       {Candidate({{0.3, 0.5, 0.0}, {0.6, 0.5, 0.0}}),
                        Candidate({{0.3, -0.5, 0.0}, {0.6, -0.5, 0.0}})},
                       {0.5F, 0.5F}, grid, Odom(), Target(), &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(2).pose().position().y(), -0.5, 1.0e-9);
}

TEST(FollowPlannerTest, PrefersLowerAbsoluteCurvatureChange) {
    auto options = Options();
    UseOnlyWeight("curvature", &options);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(planner.Select(2'000'000'000,
                               {Candidate({{0.3, 0.0, 0.0}, {0.6, 0.0, 0.0}}),
                                Candidate({{0.3, 0.0, 0.3}, {0.6, 0.0, 0.0}})},
                               {0.5F, 0.5F}, OpenGrid(), Odom(), Target(),
                               &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(1).pose().orientation().z(), 0.0, 1.0e-9);
}

TEST(FollowPlannerTest, PrefersGreaterTargetProgress) {
    auto options = Options();
    UseOnlyWeight("progress", &options);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(planner.Select(2'000'000'000,
                               {Candidate({{0.3, 0.0, 0.0}, {0.6, 0.0, 0.0}}),
                                Candidate({{0.0, 0.3, 0.0}, {0.0, 0.6, 0.0}})},
                               {0.5F, 0.5F}, OpenGrid(), Odom(),
                               Target(2.0, 0.0), &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(2).pose().position().x(), 0.6, 1.0e-9);
    EXPECT_NEAR(output.poses(2).pose().position().y(), 0.0, 1.0e-9);
}

TEST(FollowPlannerTest, PrefersAnUnblockedTargetLineOfSight) {
    auto options = Options();
    UseOnlyWeight("visibility", &options);
    auto grid = OpenGrid();
    SetCell(&grid, 1.2, 0.35, 0.0F, 0.0F, 1.0F, 0.0F);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(
        planner.Select(2'000'000'000,
                       {Candidate({{0.3, 0.3, 0.0}, {0.6, 0.6, 0.0}}),
                        Candidate({{0.3, -0.3, 0.0}, {0.6, -0.6, 0.0}})},
                       {0.5F, 0.5F}, grid, Odom(), Target(2.0, 0.0), &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(2).pose().position().y(), -0.6, 1.0e-9);
}

TEST(FollowPlannerTest, UsesLearnedScoreToBreakEqualGeometry) {
    auto options = Options();
    UseOnlyWeight("learned", &options);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(
        planner.Select(2'000'000'000,
                       {Candidate({{0.3, 0.2, 0.0}, {0.6, 0.2, 0.0}}),
                        Candidate({{0.3, -0.2, 0.0}, {0.6, -0.2, 0.0}})},
                       {0.9F, 0.1F}, OpenGrid(), Odom(), Target(), &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(2).pose().position().y(), -0.2, 1.0e-9);
}

TEST(FollowPlannerTest, UsesOriginalCandidateIndexForAnExactCostTie) {
    auto options = Options();
    UseOnlyWeight("learned", &options);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(
        planner.Select(2'000'000'000,
                       {Candidate({{0.3, 0.2, 0.0}, {0.6, 0.2, 0.0}}),
                        Candidate({{0.3, -0.2, 0.0}, {0.6, -0.2, 0.0}})},
                       {0.5F, 0.5F}, OpenGrid(), Odom(), Target(), &output));

    ASSERT_EQ(output.poses_size(), 3);
    EXPECT_NEAR(output.poses(2).pose().position().y(), 0.2, 1.0e-9);
}

TEST(FollowPlannerTest, AcceptsFiniteNegativeLearnedScoresAfterNormalization) {
    auto options = Options();
    UseOnlyWeight("learned", &options);
    const FollowPlanner planner(options);
    Path output;

    ASSERT_TRUE(planner.Select(
        2'000'000'000,
        {Candidate({{0.4, 0.2, 0.0}}), Candidate({{0.4, -0.2, 0.0}})},
        {0.25F, -0.25F}, OpenGrid(), Odom(), Target(), &output));

    ASSERT_EQ(output.poses_size(), 2);
    EXPECT_NEAR(output.poses(1).pose().position().y(), -0.2, 1.0e-9);
}

TEST(FollowPlannerTest, ReturnsStampedEmptyPathWhenEveryCandidateIsUnsafe) {
    auto grid = OpenGrid();
    SetCell(&grid, 0.5, 0.0, 0.0F, 0.0F, 1.0F, 1.0F);
    const FollowPlanner planner(Options());
    Path output;
    SeedOutput(&output);

    ASSERT_TRUE(planner.Select(2'000'000'000, {Candidate({{0.5, 0.0, 0.0}})},
                               {0.1F}, grid, Odom(), Target(), &output));

    EXPECT_EQ(output.header().frame_id(), "map");
    EXPECT_EQ(output.header().stamp().sec(), 2);
    EXPECT_EQ(output.header().stamp().nanosec(), 0U);
    EXPECT_TRUE(output.poses().empty());
}

TEST(FollowPlannerTest, RejectsNonFiniteCandidateCostAsUnsafe) {
    const FollowPlanner planner(Options());
    Path output;

    ASSERT_TRUE(planner.Select(2'000'000'000, {Candidate({{0.4, 0.0, 0.0}})},
                               {std::numeric_limits<float>::quiet_NaN()},
                               OpenGrid(), Odom(), Target(), &output));

    EXPECT_EQ(output.header().frame_id(), "map");
    EXPECT_TRUE(output.poses().empty());
}

TEST(FollowPlannerTest, MismatchedScoreCountFailsAndClearsOutput) {
    const FollowPlanner planner(Options());
    Path output;
    SeedOutput(&output);
    std::string error;

    EXPECT_FALSE(planner.Select(2'000'000'000, {Candidate({{0.4, 0.0, 0.0}})},
                                {}, OpenGrid(), Odom(), Target(), &output,
                                &error));
    EXPECT_TRUE(output.poses().empty());
    EXPECT_NE(output.header().frame_id(), "stale");
    EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
