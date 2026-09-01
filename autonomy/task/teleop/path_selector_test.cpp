/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include "autonomy/task/teleop/path_selector.hpp"
#include "autonomy/task/teleop/obstacle_grid.hpp"

#include <memory>

#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "gtest/gtest.h"

namespace autonomy::task::teleop {
namespace {

/**
 * @brief Unit tests for IntentPathSelector path library and selection
 */

using map::costmap_2d::Costmap2D;
using map::costmap_2d::FREE_SPACE;
using map::costmap_2d::LETHAL_OBSTACLE;

/**
 * @brief Create empty 10m x 10m costmap centered at origin
 */
std::shared_ptr<Costmap2D> MakeEmptyMap() {
    auto map = std::make_shared<Costmap2D>();
    map->setDefaultValue(FREE_SPACE);
    map->resizeMap(200, 200, 0.05, -5.0, -5.0);
    return map;
}

TEST(IntentPathSelectorTest, PolarLibraryGeneratesManyCandidates) {
    IntentPathSelector selector;
    PathLibraryOptions options;
    options.use_polar_spline = true;
    options.max_heading_deg = 27.0;
    selector.GenerateLibrary(options);
    EXPECT_EQ(selector.candidates().size(), 343u);
    EXPECT_EQ(selector.group_start_paths().size(), 7u);
}

TEST(IntentPathSelectorTest, PrefersForwardWhenOpen) {
    IntentPathSelector selector;
    selector.GenerateArcLib(/*num_dirs=*/9, /*num_lengths=*/3,
                                    /*max_range=*/3.0, /*ds=*/0.1);
    auto map = MakeEmptyMap();
    CostmapObstacleGrid grid(*map);
    const auto result = selector.Select(grid, 0.0, 0.5);
    ASSERT_TRUE(result.best_path.has_value());
    ASSERT_GE(result.best_path->poses_size(), 2);
    EXPECT_GT(result.best_path->poses(1).pose().position().x(),
              result.best_path->poses(0).pose().position().x());
    EXPECT_FALSE(result.feasible_indices.empty());
}

TEST(IntentPathSelectorTest, AvoidsLethalAhead) {
    IntentPathSelector selector;
    selector.set_point_per_path_thre(1);
    selector.GenerateArcLib(9, 3, 3.0, 0.1);
    auto map = MakeEmptyMap();
    for (double y = -0.4; y <= 0.4; y += 0.05) {
        unsigned int mx, my;
        ASSERT_TRUE(map->worldToMap(1.0, y, mx, my));
        map->setCost(mx, my, LETHAL_OBSTACLE);
    }
    CostmapObstacleGrid costmap_grid(*map);
    const auto result = selector.Select(costmap_grid, 0.0, 0.5);
    ASSERT_TRUE(result.best_path.has_value());
    for (const auto& ps : result.best_path->poses()) {
        unsigned int mx, my;
        if (map->worldToMap(ps.pose().position().x(), ps.pose().position().y(),
                            mx, my)) {
            EXPECT_LT(map->getCost(mx, my), LETHAL_OBSTACLE);
        }
    }
}

TEST(IntentPathSelectorTest, ClipPathStopsAtObstacle) {
    IntentPathSelector selector;
    selector.GenerateArcLib(5, 1, 2.0, 0.2);
    auto map = MakeEmptyMap();
    for (double x = 0.5; x <= 2.0; x += 0.05) {
        for (double y = -0.5; y <= 0.5; y += 0.05) {
            unsigned int mx, my;
            if (map->worldToMap(x, y, mx, my)) {
                map->setCost(mx, my, LETHAL_OBSTACLE);
            }
        }
    }
    CostmapObstacleGrid grid(*map);
    const auto& candidate = selector.candidates().front();
    const auto clipped = ClipAtObstacle(grid, candidate.path);
    ASSERT_GE(clipped.poses_size(), 1);
    EXPECT_LT(clipped.poses_size(), candidate.path.poses_size());
    EXPECT_LT(clipped.poses(clipped.poses_size() - 1).pose().position().x(), 0.6);
}

TEST(IntentPathSelectorTest, ClipForVizStopsAtMapBoundary) {
    IntentPathSelector selector;
    selector.GenerateArcLib(5, 1, 3.0, 0.2);
    auto map = MakeEmptyMap();
    CostmapObstacleGrid grid(*map);
    const auto& candidate = selector.candidates().front();
    const auto clipped = ClipForViz(grid, candidate.path, 0.02);
    ASSERT_GE(clipped.poses_size(), 2);
    for (const auto& pose : clipped.poses()) {
        EXPECT_TRUE(grid.InBounds(pose.pose().position().x(),
                                  pose.pose().position().y()));
    }
}

TEST(IntentPathSelectorTest, SelectedPathUsesBestLibraryCandidate) {
    IntentPathSelector selector;
    PathLibraryOptions options;
    options.use_polar_spline = true;
    options.use_group_start_path = false;
    options.path_range_by_speed = false;
    options.path_scale_by_speed = false;
    options.use_rot_dir_search = false;
    selector.GenerateLibrary(options);
    auto map = MakeEmptyMap();
    CostmapObstacleGrid grid(*map);
    const auto result = selector.Select(grid, 0.0, 1.0);
    ASSERT_TRUE(result.best_path.has_value());
    ASSERT_GE(result.best_index, 0);
    const auto expected = IntentPathSelector::BuildFollowPath(
        selector.candidates()[static_cast<std::size_t>(result.best_index)].path,
        result.scaled_range, result.best_rot_deg, result.best_path_scale);
    ASSERT_GE(expected.poses_size(), 2);
    ASSERT_EQ(result.best_path->poses_size(), expected.poses_size());
    const auto& got_end =
        result.best_path->poses(result.best_path->poses_size() - 1);
    const auto& exp_end = expected.poses(expected.poses_size() - 1);
    EXPECT_NEAR(got_end.pose().position().x(), exp_end.pose().position().x(),
                1e-4);
    EXPECT_NEAR(got_end.pose().position().y(), exp_end.pose().position().y(),
                1e-4);
}

TEST(IntentPathSelectorTest, PathScaleShrinksWithJoySpeed) {
    IntentPathSelector selector;
    PathLibraryOptions options;
    options.use_polar_spline = true;
    options.path_range_by_speed = true;
    options.path_scale_by_speed = true;
    options.use_rot_dir_search = false;
    selector.GenerateLibrary(options);
    auto map = MakeEmptyMap();
    CostmapObstacleGrid grid(*map);
    const auto fast = selector.Select(grid, 0.0, 1.0);
    const auto slow = selector.Select(grid, 0.0, 0.2);
    ASSERT_TRUE(fast.best_path.has_value());
    ASSERT_TRUE(slow.best_path.has_value());
    EXPECT_GT(PathArcLength(*fast.best_path), PathArcLength(*slow.best_path));
}

TEST(IntentPathSelectorTest, HasLethalHitDetectsWall) {
    IntentPathSelector selector;
    selector.GenerateArcLib(5, 1, 2.0, 0.2);
    auto map = MakeEmptyMap();
    for (double y = -0.5; y <= 0.5; y += 0.05) {
        unsigned int mx, my;
        ASSERT_TRUE(map->worldToMap(1.0, y, mx, my));
        map->setCost(mx, my, LETHAL_OBSTACLE);
    }
    CostmapObstacleGrid grid(*map);
    const auto& candidate = selector.candidates().front();
    EXPECT_TRUE(HasLethalHit(grid, candidate.path, 0.02));
}

TEST(IntentPathSelectorTest, LongPathsBeyondLocalMapStayFeasible) {
    IntentPathSelector selector;
    PathLibraryOptions options;
    options.use_polar_spline = true;
    options.path_range = 3.0;
    selector.GenerateLibrary(options);
    auto map = MakeEmptyMap();
    CostmapObstacleGrid grid(*map);
    const auto result = selector.Select(grid, 0.0, 0.5);
    EXPECT_GE(result.feasible_indices.size(), 10u);
}

TEST(IntentPathSelectorTest, OccupancyGridSelection) {
    IntentPathSelector selector;
    selector.GenerateArcLib(5, 2, 2.0, 0.1);
    automsgs::msgs::map_msgs::OccupancyGrid grid;
    grid.mutable_info()->set_resolution(0.05);
    grid.mutable_info()->set_width(200);
    grid.mutable_info()->set_height(200);
    grid.mutable_info()->mutable_origin()->mutable_position()->set_x(-5.0);
    grid.mutable_info()->mutable_origin()->mutable_position()->set_y(-5.0);
    grid.mutable_data()->Reserve(200 * 200);
    for (int i = 0; i < 200 * 200; ++i) {
        grid.add_data(0);
    }
    const auto path = selector.Select(grid, 0.0, 0.5);
    EXPECT_TRUE(path.has_value());
}

TEST(IntentPathSelectorTest, MultiScaleFallbackWhenBlocked) {
    IntentPathSelector selector;
    PathLibraryOptions options;
    options.use_polar_spline = false;
    options.num_dirs = 9;
    options.num_lengths = 3;
    options.max_range = 3.0;
    options.ds = 0.1;
    options.min_path_range = 0.8;
    options.path_range_step = 0.5;
    options.look_ahead_distance = 0.5;
    options.point_per_path_thr = 1;
    selector.GenerateLibrary(options);

    auto map = MakeEmptyMap();
    for (double y = -0.4; y <= 0.4; y += 0.05) {
        unsigned int mx, my;
        if (map->worldToMap(1.2, y, mx, my)) {
            map->setCost(mx, my, LETHAL_OBSTACLE);
        }
    }
    CostmapObstacleGrid grid(*map);
    const auto result = selector.Select(grid, 0.0, 0.5);
    ASSERT_TRUE(result.best_path.has_value());
    for (const auto& ps : result.best_path->poses()) {
        const double x = ps.pose().position().x();
        const double y = ps.pose().position().y();
        if (x > 1.0 && std::abs(y) < 0.35) {
            FAIL() << "Selected path passes through blocked corridor at x=" << x
                   << " y=" << y;
        }
    }
}

TEST(IntentPathSelectorTest, RotDirSearchEscapesBlockedForward) {
    IntentPathSelector selector;
    PathLibraryOptions options;
    options.use_polar_spline = true;
    options.use_rot_dir_search = true;
    options.num_rot_dirs = 36;
    options.max_heading_deg = 27.0;
    options.point_per_path_thr = 1;
    selector.GenerateLibrary(options);

    auto map = MakeEmptyMap();
    for (double x = 0.6; x <= 3.0; x += 0.05) {
        for (double y = -0.25; y <= 0.25; y += 0.05) {
            unsigned int mx, my;
            if (map->worldToMap(x, y, mx, my)) {
                map->setCost(mx, my, LETHAL_OBSTACLE);
            }
        }
    }
    CostmapObstacleGrid grid(*map);
    const auto result = selector.Select(grid, 0.0, 0.5);
    ASSERT_TRUE(result.best_path.has_value());
    // Forward corridor blocked — 36-dir search should pick a non-forward rotDir.
    EXPECT_NE(result.best_rot_dir, 18);
    const double end_y =
        result.best_path->poses(result.best_path->poses_size() - 1)
            .pose()
            .position()
            .y();
    EXPECT_GT(std::abs(end_y), 0.15);
}

TEST(IntentPathSelectorTest, CheckRotObstacleBlocksSideRotation) {
    IntentPathSelector selector;
    PathLibraryOptions options;
    options.use_polar_spline = true;
    options.use_rot_dir_search = true;
    options.check_rot_obstacle = true;
    options.rot_obstacle_vehicle_length = 0.6;
    options.rot_obstacle_vehicle_width = 0.4;
    options.point_per_path_thr = 1;
    selector.GenerateLibrary(options);

    auto map = MakeEmptyMap();
    for (double x = 0.15; x <= 0.55; x += 0.05) {
        for (double y = 0.05; y <= 0.45; y += 0.05) {
            unsigned int mx, my;
            if (map->worldToMap(x, y, mx, my)) {
                map->setCost(mx, my, LETHAL_OBSTACLE);
            }
        }
    }
    CostmapObstacleGrid grid(*map);
    const auto result = selector.Select(grid, 0.0, 0.5);
    ASSERT_TRUE(result.best_path.has_value());
    EXPECT_GE(result.best_rot_deg, -30.0);
}

TEST(IntentPathSelectorTest, PlotPathSetListsCurrentRotDirOnly) {
    IntentPathSelector selector;
    PathLibraryOptions options;
    options.use_polar_spline = true;
    options.use_rot_dir_search = true;
    options.plot_path_set = true;
    options.point_per_path_thr = 1;
    selector.GenerateLibrary(options);

    auto map = MakeEmptyMap();
    CostmapObstacleGrid grid(*map);
    const auto result = selector.Select(grid, 0.0, 0.5);
    ASSERT_TRUE(result.best_path.has_value());
    EXPECT_FALSE(result.free_path_indices.empty());
    for (int index : result.free_path_indices) {
        EXPECT_GE(index, 0);
        EXPECT_LT(index, static_cast<int>(selector.candidates().size()));
    }
}

TEST(IntentPathSelectorTest, FreePathIndicesRespectRgbdFovInBaseLink) {
    IntentPathSelector selector;
    PathLibraryOptions options;
    options.use_polar_spline = true;
    options.use_rot_dir_search = true;
    options.plot_path_set = true;
    options.max_heading_deg = 27.0;
    options.rgbd_hfov_deg = 90.0;
    options.rgbd_camera_offset_x = 0.03;
    options.free_paths_in_base_link = true;
    options.point_per_path_thr = 1;
    selector.GenerateLibrary(options);

    auto map = MakeEmptyMap();
    CostmapObstacleGrid grid(*map);
    const auto result = selector.Select(grid, 0.0, 0.5);
    ASSERT_TRUE(result.best_path.has_value());
    ASSERT_FALSE(result.free_path_indices.empty());

    const double half_fov = options.rgbd_hfov_deg * 0.5;
    int left = 0;
    int right = 0;
    for (int index : result.free_path_indices) {
        ASSERT_GE(index, 0);
        ASSERT_LT(index, static_cast<int>(selector.candidates().size()));
        const auto& candidate =
            selector.candidates()[static_cast<std::size_t>(index)];
        const double group_heading = IntentPathSelector::GroupHeadingDeg(
            candidate.group_id, options.max_heading_deg);
        EXPECT_LE(std::abs(group_heading), half_fov + 1.0);
        automsgs::msgs::nav_msgs::Path path = candidate.path;
        path = IntentPathSelector::TrimPathToArcLength(path, result.scaled_range);
        path = IntentPathSelector::RotateAndScalePath(path, 0.0,
                                                      result.best_path_scale);
        ASSERT_GE(path.poses_size(), 2);
        const auto& end = path.poses(path.poses_size() - 1).pose().position();
        if (end.y() > 0.01) {
            ++left;
        } else if (end.y() < -0.01) {
            ++right;
        }
    }
    EXPECT_GT(left, 0);
    EXPECT_GT(right, 0);
    EXPECT_NEAR(left, right, 8);
}

TEST(IntentPathSelectorTest, ZeroSpeedReturnsEmptySelection) {
    IntentPathSelector selector;
    selector.GenerateArcLib(5, 2, 2.0, 0.1);
    auto map = MakeEmptyMap();
    CostmapObstacleGrid grid(*map);
    const auto result = selector.Select(grid, 0.0, 0.0);
    EXPECT_FALSE(result.best_path.has_value());
}

}  // namespace
}  // namespace autonomy::task::teleop
