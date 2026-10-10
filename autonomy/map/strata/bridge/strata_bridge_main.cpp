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
#include <string>

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/cli_options.hpp"
#include "autonomy/map/strata/bridge/strata_bridge_node.hpp"

int main(int argc, char** argv) {
    std::string channel_prefix = "/strata";
    std::string frame_id = "map";
    double publish_rate_hz = 10.0;
    std::string slam_image_path;
    double slam_start_x = 0.0;
    double slam_start_y = 0.0;
    int slam_x_grid_count = 0;
    int slam_y_grid_count = 0;
    double slam_resolution = 0.05;
    double map_center_lng = 116.4074;
    double map_center_lat = 39.9042;
    double map_zoom = 16.0;

    bool enable_robot_sim = false;
    double map_width_meters = 10.0;
    double map_height_meters = 10.0;
    std::string demo_robot_id = "demo-robot";
    std::string demo_robot_name = "Demo Robot";
    double robot_start_x = 1.0;
    double robot_start_y = 1.0;
    double robot_goal_x = 9.0;
    double robot_goal_y = 9.0;
    bool auto_move_on_start = false;
    double robot_tick_hz = 20.0;
    bool seed_demo_forbidden_zone = true;

    CLI::App app{"Strata map bridge (Autolink)."};
    app.add_option("--channel_prefix", channel_prefix, "Autolink channel prefix for strata topics.")
        ->capture_default_str();
    app.add_option("--frame_id", frame_id, "Frame id used in exported scene messages.")
        ->capture_default_str();
    app.add_option("--publish_rate_hz", publish_rate_hz,
                   "Maximum publish rate when scene revision changes.")
        ->capture_default_str();
    app.add_option("--slam_image_path", slam_image_path, "Optional SLAM raster image path.")
        ->capture_default_str();
    app.add_option("--slam_start_x", slam_start_x, "SLAM map origin X.")
        ->capture_default_str();
    app.add_option("--slam_start_y", slam_start_y, "SLAM map origin Y.")
        ->capture_default_str();
    app.add_option("--slam_x_grid_count", slam_x_grid_count, "SLAM map width in cells.")
        ->capture_default_str();
    app.add_option("--slam_y_grid_count", slam_y_grid_count, "SLAM map height in cells.")
        ->capture_default_str();
    app.add_option("--slam_resolution", slam_resolution, "SLAM map resolution in meters.")
        ->capture_default_str();
    app.add_option("--map_center_lng", map_center_lng, "Map view center longitude.")
        ->capture_default_str();
    app.add_option("--map_center_lat", map_center_lat, "Map view center latitude.")
        ->capture_default_str();
    app.add_option("--map_zoom", map_zoom, "Map view zoom.")
        ->capture_default_str();
    app.add_flag("--enable_robot_sim", enable_robot_sim,
                 "Enable RobotEngine + Pathfinder simulation loop.");
    app.add_option("--map_width_meters", map_width_meters,
                   "Map width in meters for pathfinding (used without SLAM).")
        ->capture_default_str();
    app.add_option("--map_height_meters", map_height_meters,
                   "Map height in meters for pathfinding (used without SLAM).")
        ->capture_default_str();
    app.add_option("--demo_robot_id", demo_robot_id, "Robot id for simulation demo.")
        ->capture_default_str();
    app.add_option("--demo_robot_name", demo_robot_name, "Display name for simulation robot.")
        ->capture_default_str();
    app.add_option("--robot_start_x", robot_start_x, "Demo robot initial X (meters).")
        ->capture_default_str();
    app.add_option("--robot_start_y", robot_start_y, "Demo robot initial Y (meters).")
        ->capture_default_str();
    app.add_option("--robot_goal_x", robot_goal_x, "Auto-move goal X when --auto_move_on_start.")
        ->capture_default_str();
    app.add_option("--robot_goal_y", robot_goal_y, "Auto-move goal Y when --auto_move_on_start.")
        ->capture_default_str();
    app.add_flag("--auto_move_on_start", auto_move_on_start,
                 "Plan path and move demo robot on startup.");
    app.add_option("--robot_tick_hz", robot_tick_hz, "RobotEngine tick rate.")
        ->capture_default_str();
    app.add_option("--seed_demo_forbidden_zone", seed_demo_forbidden_zone,
                   "Add central forbidden zone when no semantic zones exist.")
        ->default_val(true);

    autonomy::common::ParseOrExit(app, argc, argv);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed.";
        return EXIT_FAILURE;
    }

    autonomy::map::strata::bridge::StrataBridgeOptions options;
    options.channel_prefix = channel_prefix;
    options.frame_id = frame_id;
    options.publish_rate_hz = publish_rate_hz;
    options.map_view.center.x = map_center_lng;
    options.map_view.center.y = map_center_lat;
    options.map_view.zoom = map_zoom;
    options.slam_map.startX = slam_start_x;
    options.slam_map.startY = slam_start_y;
    options.slam_map.xGridCount = slam_x_grid_count;
    options.slam_map.yGridCount = slam_y_grid_count;
    options.slam_map.resolution = slam_resolution;
    options.slam_map.imagePath = slam_image_path;
    options.load_slam_map = !slam_image_path.empty() && slam_x_grid_count > 0 && slam_y_grid_count > 0;
    options.enable_robot_sim = enable_robot_sim;
    options.map_width_meters = map_width_meters;
    options.map_height_meters = map_height_meters;
    options.demo_robot_id = demo_robot_id;
    options.demo_robot_name = demo_robot_name;
    options.robot_start_x = robot_start_x;
    options.robot_start_y = robot_start_y;
    options.robot_goal_x = robot_goal_x;
    options.robot_goal_y = robot_goal_y;
    options.auto_move_on_start = auto_move_on_start;
    options.robot_tick_hz = robot_tick_hz;
    options.seed_demo_forbidden_zone = seed_demo_forbidden_zone;

    auto node = autolink::CreateNode("strata_bridge");
    autonomy::map::strata::bridge::StrataBridgeNode bridge(std::move(options));
    if (!bridge.Init(node)) {
        LOG(ERROR) << "Failed to initialize strata bridge node.";
        autolink::Clear();
        return EXIT_FAILURE;
    }

    bridge.SpinUntilShutdown();
    autolink::Clear();
    return EXIT_SUCCESS;
}
