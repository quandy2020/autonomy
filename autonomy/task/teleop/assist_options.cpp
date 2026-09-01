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

#include "autonomy/task/teleop/assist_options.hpp"

#include <cstdint>
#include <cstdlib>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/task/teleop/constants.hpp"

namespace autonomy::task::teleop {
namespace {

using ::autonomy::common::ConfigurationFileResolver;
using ::autonomy::common::ConfigurationSearchDirectories;
using ::autonomy::common::LuaParameterDictionary;

/**
 * @brief Parse point cloud feeder options from Lua dict
 */
PointCloudObstacleFeeder::Options LoadPointCloudOptions(
    LuaParameterDictionary* const dict) {
    PointCloudObstacleFeeder::Options options;
    options.cloud_topic.clear();
    if (dict->HasKey("cloud_topic")) {
        options.cloud_topic = dict->GetString("cloud_topic");
    }
    if (dict->HasKey("depth_topic")) {
        options.depth_topic = dict->GetString("depth_topic");
    }
    if (dict->HasKey("camera_info_topic")) {
        options.camera_info_topic = dict->GetString("camera_info_topic");
    }
    if (dict->HasKey("depth_decimation")) {
        options.depth_decimation =
            static_cast<int>(dict->GetDouble("depth_decimation"));
    }
    if (dict->HasKey("min_depth_m")) {
        options.min_depth_m = dict->GetDouble("min_depth_m");
    }
    if (dict->HasKey("max_depth_m")) {
        options.max_depth_m = dict->GetDouble("max_depth_m");
    }
    if (dict->HasKey("stale_timeout_sec")) {
        options.stale_timeout_sec = dict->GetDouble("stale_timeout_sec");
    }
    return options;
}

/**
 * @brief Parse laser scan options from Lua dict
 */
TeleopMppiAssist::LaserScanOptions LoadLaserScanOptions(
    LuaParameterDictionary* const dict) {
    TeleopMppiAssist::LaserScanOptions options;
    if (dict->HasKey("topic")) {
        options.topic = dict->GetString("topic");
    }
    if (dict->HasKey("stale_timeout_sec")) {
        options.stale_timeout_sec = dict->GetDouble("stale_timeout_sec");
    }
    return options;
}

/**
 * @brief Parse path library options from Lua dict
 */
PathLibraryOptions LoadPathLibraryOptions(LuaParameterDictionary* const dict) {
    PathLibraryOptions options;
    if (dict->HasKey("use_polar_spline")) {
        options.use_polar_spline = dict->GetBool("use_polar_spline");
    }
    if (dict->HasKey("segment_length")) {
        options.segment_length = dict->GetDouble("segment_length");
    }
    if (dict->HasKey("max_heading_deg")) {
        options.max_heading_deg = dict->GetDouble("max_heading_deg");
    }
    if (dict->HasKey("hierarchy_scale")) {
        options.hierarchy_scale = dict->GetDouble("hierarchy_scale");
    }
    if (dict->HasKey("path_range")) {
        options.path_range = dict->GetDouble("path_range");
    }
    if (dict->HasKey("sample_ds")) {
        options.sample_ds = dict->GetDouble("sample_ds");
    }
    if (dict->HasKey("library_knot_ds")) {
        options.library_knot_ds = dict->GetDouble("library_knot_ds");
    }
    if (dict->HasKey("sensor_range")) {
        options.sensor_range = dict->GetDouble("sensor_range");
    }
    if (dict->HasKey("use_group_start_path")) {
        options.use_group_start_path = dict->GetBool("use_group_start_path");
    }
    if (dict->HasKey("min_path_range")) {
        options.min_path_range = dict->GetDouble("min_path_range");
    }
    if (dict->HasKey("path_range_step")) {
        options.path_range_step = dict->GetDouble("path_range_step");
    }
    if (dict->HasKey("look_ahead_distance")) {
        options.look_ahead_distance = dict->GetDouble("look_ahead_distance");
    }
    if (dict->HasKey("use_group_selection")) {
        options.use_group_selection = dict->GetBool("use_group_selection");
    }
    if (dict->HasKey("point_per_path_thr")) {
        options.point_per_path_thr =
            static_cast<int>(dict->GetDouble("point_per_path_thr"));
    }
    if (dict->HasKey("dir_weight")) {
        options.dir_weight = dict->GetDouble("dir_weight");
    }
    if (dict->HasKey("dir_threshold_deg")) {
        options.dir_threshold_deg = dict->GetDouble("dir_threshold_deg");
    }
    if (dict->HasKey("path_range_by_speed")) {
        options.path_range_by_speed = dict->GetBool("path_range_by_speed");
    }
    if (dict->HasKey("max_linear_speed")) {
        options.max_linear_speed = dict->GetDouble("max_linear_speed");
    }
    if (dict->HasKey("use_rot_dir_search")) {
        options.use_rot_dir_search = dict->GetBool("use_rot_dir_search");
    }
    if (dict->HasKey("num_rot_dirs")) {
        options.num_rot_dirs = static_cast<int>(dict->GetDouble("num_rot_dirs"));
    }
    if (dict->HasKey("def_path_scale")) {
        options.def_path_scale = dict->GetDouble("def_path_scale");
    }
    if (dict->HasKey("min_path_scale")) {
        options.min_path_scale = dict->GetDouble("min_path_scale");
    }
    if (dict->HasKey("path_scale_step")) {
        options.path_scale_step = dict->GetDouble("path_scale_step");
    }
    if (dict->HasKey("path_scale_by_speed")) {
        options.path_scale_by_speed = dict->GetBool("path_scale_by_speed");
    }
    if (dict->HasKey("dir_to_vehicle")) {
        options.dir_to_vehicle = dict->GetBool("dir_to_vehicle");
    }
    if (dict->HasKey("check_rot_obstacle")) {
        options.check_rot_obstacle = dict->GetBool("check_rot_obstacle");
    }
    if (dict->HasKey("rot_obstacle_vehicle_length")) {
        options.rot_obstacle_vehicle_length =
            dict->GetDouble("rot_obstacle_vehicle_length");
    }
    if (dict->HasKey("rot_obstacle_vehicle_width")) {
        options.rot_obstacle_vehicle_width =
            dict->GetDouble("rot_obstacle_vehicle_width");
    }
    if (dict->HasKey("two_way_drive")) {
        options.two_way_drive = dict->GetBool("two_way_drive");
    }
    if (dict->HasKey("clearance_weight")) {
        options.clearance_weight = dict->GetDouble("clearance_weight");
    }
    if (dict->HasKey("smoothness_weight")) {
        options.smoothness_weight = dict->GetDouble("smoothness_weight");
    }
    if (dict->HasKey("efficiency_weight")) {
        options.efficiency_weight = dict->GetDouble("efficiency_weight");
    }
    if (dict->HasKey("velocity_continuity_weight")) {
        options.velocity_continuity_weight =
            dict->GetDouble("velocity_continuity_weight");
    }
    if (dict->HasKey("temporal_weight")) {
        options.temporal_weight = dict->GetDouble("temporal_weight");
    }
    if (dict->HasKey("normalize_group_scores")) {
        options.normalize_group_scores = dict->GetBool("normalize_group_scores");
    }
    if (dict->HasKey("traversability_weight")) {
        options.traversability_weight = dict->GetDouble("traversability_weight");
    }
    if (dict->HasKey("plot_path_set")) {
        options.plot_path_set = dict->GetBool("plot_path_set");
    }
    if (dict->HasKey("rgbd_hfov_deg")) {
        options.rgbd_hfov_deg = dict->GetDouble("rgbd_hfov_deg");
    }
    if (dict->HasKey("rgbd_camera_offset_x")) {
        options.rgbd_camera_offset_x = dict->GetDouble("rgbd_camera_offset_x");
    }
    if (dict->HasKey("rgbd_camera_offset_y")) {
        options.rgbd_camera_offset_y = dict->GetDouble("rgbd_camera_offset_y");
    }
    if (dict->HasKey("free_paths_in_base_link")) {
        options.free_paths_in_base_link = dict->GetBool("free_paths_in_base_link");
    }
    if (dict->HasKey("free_paths_plot_library_fan")) {
        options.free_paths_plot_library_fan =
            dict->GetBool("free_paths_plot_library_fan");
    }
    if (dict->HasKey("free_paths_max_markers")) {
        options.free_paths_max_markers =
            static_cast<int>(dict->GetDouble("free_paths_max_markers"));
    }
    if (dict->HasKey("free_paths_filter_collisions")) {
        options.free_paths_filter_collisions =
            dict->GetBool("free_paths_filter_collisions");
    }
    if (dict->HasKey("goal_cost")) {
        options.goal_cost = dict->GetDouble("goal_cost");
    }
    if (dict->HasKey("num_dirs")) {
        options.num_dirs = static_cast<int>(dict->GetDouble("num_dirs"));
    }
    if (dict->HasKey("num_lengths")) {
        options.num_lengths = static_cast<int>(dict->GetDouble("num_lengths"));
    }
    if (dict->HasKey("max_range")) {
        options.max_range = dict->GetDouble("max_range");
    }
    if (dict->HasKey("ds")) {
        options.ds = dict->GetDouble("ds");
    }
    return options;
}

/**
 * @brief Apply local costmap overrides from Lua dict
 */
void LoadCostmapOverrides(LuaParameterDictionary* const dict,
                               map::proto::Costmap2DOptions* out) {
    if (dict == nullptr || out == nullptr) {
        return;
    }
    if (dict->HasKey("width")) {
        out->set_width(static_cast<int32_t>(dict->GetDouble("width")));
    }
    if (dict->HasKey("height")) {
        out->set_height(static_cast<int32_t>(dict->GetDouble("height")));
    }
    if (dict->HasKey("resolution")) {
        out->set_resolution(dict->GetDouble("resolution"));
    }
    if (dict->HasKey("update_frequency")) {
        out->set_update_frequency(dict->GetDouble("update_frequency"));
    }
    if (dict->HasKey("robot_radius")) {
        out->set_robot_radius(dict->GetDouble("robot_radius"));
    }
}

/**
 * @brief Load point cloud options from dict aliases
 */
void LoadPointCloudFromDict(LuaParameterDictionary* dict,
                             PointCloudObstacleFeeder::Options* out) {
    if (dict->HasKey("point_cloud")) {
        *out = LoadPointCloudOptions(dict->GetDictionary("point_cloud").get());
    } else if (dict->HasKey("rgbd")) {
        *out = LoadPointCloudOptions(dict->GetDictionary("rgbd").get());
    }
}

}  // namespace

/**
 * @brief Load full teleop assist options from Lua config file
 */
TeleopMppiAssist::Options LoadTeleopAssistOptions(
    const std::string& config_directory, const std::string& relative_path) {
    TeleopMppiAssist::Options options;
    options.enabled = false;

    const char* env_path = std::getenv("TELEOP_ASSIST_CONFIG");
    const std::string config_rel =
        (env_path != nullptr && env_path[0] != '\0') ? env_path : relative_path;

    try {
        const auto dirs = ConfigurationSearchDirectories(config_directory);
        auto file_resolver = std::make_unique<ConfigurationFileResolver>(dirs);
        const std::string code =
            file_resolver->GetFileContentOrDie(config_rel);
        LuaParameterDictionary dict(code, std::move(file_resolver));

        if (dict.HasKey("assist_enabled")) {
            options.enabled = dict.GetBool("assist_enabled");
        }
        if (dict.HasKey("publish_path_viz")) {
            options.publish_path_viz = dict.GetBool("publish_path_viz");
        }
        if (dict.HasKey("path_viz_rate_hz")) {
            options.path_viz_rate_hz = dict.GetDouble("path_viz_rate_hz");
        }
        if (dict.HasKey("global_frame")) {
            const std::string frame = dict.GetString("global_frame");
            if (frame != kDefaultBaseFrame) {
                AWARN << "Teleop assist ignores global_frame=\"" << frame
                      << "\"; teleop uses local " << kDefaultBaseFrame
                      << " only";
            }
        }
        if (dict.HasKey("angular_to_dir_gain")) {
            options.angular_to_dir_gain = dict.GetDouble("angular_to_dir_gain");
        }
        if (dict.HasKey("stopped_linear_epsilon")) {
            options.stopped_linear_epsilon =
                dict.GetDouble("stopped_linear_epsilon");
        }
        if (dict.HasKey("in_place_angular_scale")) {
            options.in_place_angular_scale =
                dict.GetDouble("in_place_angular_scale");
        }
        if (dict.HasKey("blocked_turn_enabled")) {
            options.blocked_turn_enabled = dict.GetBool("blocked_turn_enabled");
        }
        if (dict.HasKey("blocked_turn_probe_length")) {
            options.blocked_turn_probe_length =
                dict.GetDouble("blocked_turn_probe_length");
        }
        if (dict.HasKey("blocked_turn_max_angular")) {
            options.blocked_turn_max_angular =
                dict.GetDouble("blocked_turn_max_angular");
        }
        if (dict.HasKey("stale_cloud_timeout_sec")) {
            options.point_cloud.stale_timeout_sec =
                dict.GetDouble("stale_cloud_timeout_sec");
        }
        if (dict.HasKey("use_laser_scan")) {
            options.use_laser_for_costmap = dict.GetBool("use_laser_scan");
        }
        if (dict.HasKey("laser_scan")) {
            options.laser_scan = LoadLaserScanOptions(
                dict.GetDictionary("laser_scan").get());
        }
        LoadPointCloudFromDict(&dict, &options.point_cloud);
        if (dict.HasKey("path_library")) {
            options.path_library = LoadPathLibraryOptions(
                dict.GetDictionary("path_library").get());
        }
        if (dict.HasKey("local_costmap")) {
            LoadCostmapOverrides(
                dict.GetDictionary("local_costmap").get(), &options.costmap);
        }
    } catch (const std::exception& ex) {
        AINFO << "Teleop assist config not loaded (" << config_rel
              << "): " << ex.what() << " — assist disabled";
        options.enabled = false;
    }

    return options;
}

}  // namespace autonomy::task::teleop
