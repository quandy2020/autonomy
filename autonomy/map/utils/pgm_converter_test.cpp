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

#include "autonomy/map/utils/pgm_converter.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace map {
namespace utils {
namespace {

// Helper function to create a simple test costmap.
std::shared_ptr<map::costmap_2d::Costmap2D> CreateTestCostmap(
    unsigned int width, unsigned int height, double resolution) {
    automsgs::msgs::map_msgs::OccupancyGrid grid;
    grid.mutable_info()->set_width(width);
    grid.mutable_info()->set_height(height);
    grid.mutable_info()->set_resolution(resolution);
    grid.mutable_info()->mutable_origin()->mutable_position()->set_x(0.0);
    grid.mutable_info()->mutable_origin()->mutable_position()->set_y(0.0);
    grid.mutable_info()->mutable_origin()->mutable_position()->set_z(0.0);
    grid.mutable_info()->mutable_origin()->mutable_orientation()->set_w(1.0);

    const size_t size = width * height;
    grid.mutable_data()->Resize(static_cast<int>(size),
                                map::costmap_2d::utils::OCC_GRID_FREE);

    // Add some obstacles in the center
    for (unsigned int y = height / 3; y < 2 * height / 3; ++y) {
        for (unsigned int x = width / 3; x < 2 * width / 3; ++x) {
            grid.set_data(static_cast<int>(y * width + x),
                          map::costmap_2d::utils::OCC_GRID_OCCUPIED);
        }
    }

    return std::make_shared<map::costmap_2d::Costmap2D>(grid);
}

// Helper function to create a simple test path.
automsgs::msgs::nav_msgs::Path CreateTestPath(double start_x, double start_y,
                                            double goal_x, double goal_y,
                                            int num_points) {
    automsgs::msgs::nav_msgs::Path path;
    for (int i = 0; i < num_points; ++i) {
        automsgs::msgs::geometry_msgs::PoseStamped pose;
        pose.mutable_header()->set_frame_id("map");
        double t = static_cast<double>(i) / (num_points - 1);
        pose.mutable_pose()->mutable_position()->set_x(start_x + (goal_x - start_x) * t);
        pose.mutable_pose()->mutable_position()->set_y(start_y + (goal_y - start_y) * t);
        pose.mutable_pose()->mutable_position()->set_z(0.0);
        pose.mutable_pose()->mutable_orientation()->set_w(1.0);
        *path.mutable_poses()->Add() = pose;
    }
    return path;
}

TEST(PgmConverterTest, LoadFromPgmInvalidPath) {
    auto costmap = PgmConverter::loadFromPgm("/nonexistent/path/to/file.pgm");
    EXPECT_EQ(costmap, nullptr);
}

TEST(PgmConverterTest, LoadFromYamlInvalidPath) {
    auto costmap = PgmConverter::loadFromYaml("/nonexistent/path/to/file.yaml");
    EXPECT_EQ(costmap, nullptr);
}

TEST(PgmConverterTest, SavePathToImageSuccess) {
    const unsigned int width = 100;
    const unsigned int height = 100;
    const double resolution = 0.05;
    auto costmap = CreateTestCostmap(width, height, resolution);

    auto path = CreateTestPath(1.0, 1.0, 3.0, 3.0, 10);

    const std::string output_path = "/tmp/test_path_output.png";
    PgmConverter::RenderParameters params;
    bool success =
        PgmConverter::savePathToImage(*costmap, path, output_path, params);

    EXPECT_TRUE(success);

    // Verify file was created
    std::ifstream file(output_path);
    EXPECT_TRUE(file.good());
    file.close();

    // Clean up
    std::remove(output_path.c_str());
}

TEST(PgmConverterTest, SavePathToImageEmptyPath) {
    auto costmap = CreateTestCostmap(50, 50, 0.1);
    automsgs::msgs::nav_msgs::Path empty_path;

    const std::string output_path = "/tmp/test_empty_path.png";
    bool success =
        PgmConverter::savePathToImage(*costmap, empty_path, output_path);

    EXPECT_TRUE(success);

    std::ifstream file(output_path);
    EXPECT_TRUE(file.good());
    file.close();
    std::remove(output_path.c_str());
}

TEST(PgmConverterTest, SavePathToImageJpegFormat) {
    auto costmap = CreateTestCostmap(80, 80, 0.05);
    auto path = CreateTestPath(0.5, 0.5, 2.5, 2.5, 5);

    const std::string output_path = "/tmp/test_path_jpeg";
    PgmConverter::RenderParameters params;
    params.output_format = "jpg";
    bool success =
        PgmConverter::savePathToImage(*costmap, path, output_path, params);

    EXPECT_TRUE(success);

    std::ifstream file(output_path + ".jpeg");
    EXPECT_TRUE(file.good());
    file.close();
    std::remove((output_path + ".jpeg").c_str());
}

TEST(PgmConverterTest, SavePathToImageWithMarkers) {
    auto costmap = CreateTestCostmap(100, 100, 0.05);
    auto path = CreateTestPath(1.0, 1.0, 4.0, 4.0, 8);

    const std::string output_path = "/tmp/test_path_markers.png";
    PgmConverter::RenderParameters params;
    params.draw_start_marker = true;
    params.draw_goal_marker = true;
    params.draw_path_points = true;
    params.start_marker_size = 8.0;
    params.goal_marker_size = 8.0;
    params.path_line_width = 3.0;

    bool success =
        PgmConverter::savePathToImage(*costmap, path, output_path, params);

    EXPECT_TRUE(success);

    std::ifstream file(output_path);
    EXPECT_TRUE(file.good());
    file.close();
    std::remove(output_path.c_str());
}

TEST(PgmConverterTest, SavePathToImageWithoutMarkers) {
    auto costmap = CreateTestCostmap(60, 60, 0.1);
    auto path = CreateTestPath(0.5, 0.5, 5.0, 5.0, 6);

    const std::string output_path = "/tmp/test_path_no_markers.png";
    PgmConverter::RenderParameters params;
    params.draw_start_marker = false;
    params.draw_goal_marker = false;
    params.draw_path_points = false;

    bool success =
        PgmConverter::savePathToImage(*costmap, path, output_path, params);

    EXPECT_TRUE(success);

    std::ifstream file(output_path);
    EXPECT_TRUE(file.good());
    file.close();
    std::remove(output_path.c_str());
}

TEST(PgmConverterTest, SavePathToImageSinglePoint) {
    auto costmap = CreateTestCostmap(50, 50, 0.1);
    auto path = CreateTestPath(2.0, 2.0, 2.0, 2.0, 1);

    const std::string output_path = "/tmp/test_single_point.png";
    bool success = PgmConverter::savePathToImage(*costmap, path, output_path);

    EXPECT_TRUE(success);
    std::ifstream file(output_path);
    EXPECT_TRUE(file.good());
    file.close();
    std::remove(output_path.c_str());
}

TEST(PgmConverterTest, LoadParametersDefaultValues) {
    PgmConverter::LoadParameters params;
    EXPECT_DOUBLE_EQ(params.resolution, 0.05);
    EXPECT_DOUBLE_EQ(params.origin_x, 0.0);
    EXPECT_DOUBLE_EQ(params.origin_y, 0.0);
    EXPECT_DOUBLE_EQ(params.origin_yaw, 0.0);
    EXPECT_DOUBLE_EQ(params.free_thresh, 0.196);
    EXPECT_DOUBLE_EQ(params.occupied_thresh, 0.65);
    EXPECT_FALSE(params.negate);
}

TEST(PgmConverterTest, RenderParametersDefaultValues) {
    PgmConverter::RenderParameters params;
    EXPECT_EQ(params.output_format, "png");
    EXPECT_DOUBLE_EQ(params.path_line_width, 2.0);
    EXPECT_DOUBLE_EQ(params.start_marker_size, 5.0);
    EXPECT_DOUBLE_EQ(params.goal_marker_size, 5.0);
    EXPECT_TRUE(params.draw_start_marker);
    EXPECT_TRUE(params.draw_goal_marker);
    EXPECT_FALSE(params.draw_path_points);
}

TEST(PgmConverterTest, SavePathToImageLargeCostmap) {
    auto costmap = CreateTestCostmap(500, 500, 0.02);
    auto path = CreateTestPath(1.0, 1.0, 8.0, 8.0, 20);

    const std::string output_path = "/tmp/test_large_costmap.png";
    bool success = PgmConverter::savePathToImage(*costmap, path, output_path);

    EXPECT_TRUE(success);
    std::ifstream file(output_path);
    EXPECT_TRUE(file.good());
    file.close();
    std::remove(output_path.c_str());
}

TEST(PgmConverterTest, SavePathToImagePathOutsideBounds) {
    auto costmap = CreateTestCostmap(50, 50, 0.1);
    // Create path with points outside the costmap bounds
    automsgs::msgs::nav_msgs::Path path;
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_header()->set_frame_id("map");
    pose.mutable_pose()->mutable_position()->set_x(100.0);  // Way outside bounds
    pose.mutable_pose()->mutable_position()->set_y(100.0);
    pose.mutable_pose()->mutable_position()->set_z(0.0);
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    *path.mutable_poses()->Add() = pose;

    const std::string output_path = "/tmp/test_path_outside.png";
    bool success = PgmConverter::savePathToImage(*costmap, path, output_path);

    EXPECT_TRUE(success);
    std::ifstream file(output_path);
    EXPECT_TRUE(file.good());
    file.close();
    std::remove(output_path.c_str());
}

}  // namespace
}  // namespace utils
}  // namespace map
}  // namespace autonomy
