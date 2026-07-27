/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/map/strata/bridge/strata_bridge_node.hpp"

DEFINE_string(channel_prefix, "/strata", "Autolink channel prefix for strata topics.");
DEFINE_string(frame_id, "map", "Frame id used in exported scene messages.");
DEFINE_double(publish_rate_hz, 10.0, "Maximum publish rate when scene revision changes.");
DEFINE_string(slam_image_path, "", "Optional SLAM raster image path.");
DEFINE_double(slam_start_x, 0.0, "SLAM map origin X.");
DEFINE_double(slam_start_y, 0.0, "SLAM map origin Y.");
DEFINE_int32(slam_x_grid_count, 0, "SLAM map width in cells.");
DEFINE_int32(slam_y_grid_count, 0, "SLAM map height in cells.");
DEFINE_double(slam_resolution, 0.05, "SLAM map resolution in meters.");
DEFINE_double(map_center_lng, 116.4074, "Map view center longitude.");
DEFINE_double(map_center_lat, 39.9042, "Map view center latitude.");
DEFINE_double(map_zoom, 16.0, "Map view zoom.");

DEFINE_bool(enable_robot_sim, false, "Enable RobotEngine + Pathfinder simulation loop.");
DEFINE_double(map_width_meters, 10.0, "Map width in meters for pathfinding (used without SLAM).");
DEFINE_double(map_height_meters, 10.0, "Map height in meters for pathfinding (used without SLAM).");
DEFINE_string(demo_robot_id, "demo-robot", "Robot id for simulation demo.");
DEFINE_string(demo_robot_name, "Demo Robot", "Display name for simulation robot.");
DEFINE_double(robot_start_x, 1.0, "Demo robot initial X (meters).");
DEFINE_double(robot_start_y, 1.0, "Demo robot initial Y (meters).");
DEFINE_double(robot_goal_x, 9.0, "Auto-move goal X when --auto_move_on_start.");
DEFINE_double(robot_goal_y, 9.0, "Auto-move goal Y when --auto_move_on_start.");
DEFINE_bool(auto_move_on_start, false, "Plan path and move demo robot on startup.");
DEFINE_double(robot_tick_hz, 20.0, "RobotEngine tick rate.");
DEFINE_bool(seed_demo_forbidden_zone, true, "Add central forbidden zone when no semantic zones exist.");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed.";
        return EXIT_FAILURE;
    }

    autonomy::map::strata::bridge::StrataBridgeOptions options;
    options.channel_prefix = FLAGS_channel_prefix;
    options.frame_id = FLAGS_frame_id;
    options.publish_rate_hz = FLAGS_publish_rate_hz;
    options.map_view.center.x = FLAGS_map_center_lng;
    options.map_view.center.y = FLAGS_map_center_lat;
    options.map_view.zoom = FLAGS_map_zoom;
    options.slam_map.startX = FLAGS_slam_start_x;
    options.slam_map.startY = FLAGS_slam_start_y;
    options.slam_map.xGridCount = FLAGS_slam_x_grid_count;
    options.slam_map.yGridCount = FLAGS_slam_y_grid_count;
    options.slam_map.resolution = FLAGS_slam_resolution;
    options.slam_map.imagePath = FLAGS_slam_image_path;
    options.load_slam_map = !FLAGS_slam_image_path.empty() && FLAGS_slam_x_grid_count > 0 &&
                            FLAGS_slam_y_grid_count > 0;
    options.enable_robot_sim = FLAGS_enable_robot_sim;
    options.map_width_meters = FLAGS_map_width_meters;
    options.map_height_meters = FLAGS_map_height_meters;
    options.demo_robot_id = FLAGS_demo_robot_id;
    options.demo_robot_name = FLAGS_demo_robot_name;
    options.robot_start_x = FLAGS_robot_start_x;
    options.robot_start_y = FLAGS_robot_start_y;
    options.robot_goal_x = FLAGS_robot_goal_x;
    options.robot_goal_y = FLAGS_robot_goal_y;
    options.auto_move_on_start = FLAGS_auto_move_on_start;
    options.robot_tick_hz = FLAGS_robot_tick_hz;
    options.seed_demo_forbidden_zone = FLAGS_seed_demo_forbidden_zone;

    auto node = autolink::CreateNode("strata_bridge");
    autonomy::map::strata::bridge::StrataBridgeNode bridge(std::move(options));
    if (!bridge.Init(node)) {
        LOG(ERROR) << "Failed to initialize strata bridge node.";
        autolink::Clear();
        return EXIT_FAILURE;
    }

    bridge.SpinUntilShutdown();
    autolink::Clear();
    google::ShutdownGoogleLogging();
    return EXIT_SUCCESS;
}
