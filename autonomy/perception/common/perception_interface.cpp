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

#include "autonomy/perception/common/perception_interface.hpp"

#include "autonomy/perception/exploration/common/types.hpp"

namespace autonomy {
namespace perception {
namespace common {
namespace {

using ::autonomy::common::LuaParameterDictionary;

void SetStringIfPresent(LuaParameterDictionary* dict, const std::string& key,
                        std::string* out) {
  if (dict->HasKey(key)) {
    *out = dict->GetString(key);
  }
}

}  // namespace

proto::PerceptionOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
  proto::PerceptionOptions options;
  options.set_enabled(false);
  options.set_enable_rgbd_exploration(false);
  options.set_exploration_config("perception/exploration_rgbd_tare.lua");
  options.set_odom_topic("/odom");
  options.set_depth_topic("/camera/depth/image_raw");
  options.set_camera_info_topic("/camera/depth/camera_info");
  options.set_camera_frame("camera_depth_optical_frame");
  options.set_map_frame("map");
  options.set_planner_hz(2.0);
  options.set_path_topic(exploration::kExplorationPathTopic);
  options.set_waypoint_topic(exploration::kExplorationWaypointTopic);
  options.set_map_topic(exploration::kExplorationMapTopic);
  options.set_explorer_backend("rgbd_tare");
  options.set_point_cloud_topic("/velodyne_points");
  options.set_global_path_topic(exploration::kExplorationGlobalPathTopic);
  options.set_local_path_topic(exploration::kExplorationLocalPathTopic);
  options.set_exploration_finished_topic(exploration::kExplorationFinishedTopic);
  options.set_exploration_progress_topic(exploration::kExplorationProgressTopic);
  options.set_navigation_boundary_topic(exploration::kNavigationBoundaryTopic);
  options.set_terrain_map_topic("/terrain_map");
  options.set_enable_exploration_nav_bridge(true);
  options.set_prior_map_topic("/map");
  options.set_vg_markers_topic(exploration::kExplorationVgMarkersTopic);
  options.set_enable_yopo_track(false);
  options.set_track_config("perception/track_yopo.lua");
  options.set_cmd_vel_topic("/cmd_vel");

  if (parameter_dictionary == nullptr) {
    return options;
  }
  if (parameter_dictionary->HasKey("enabled")) {
    options.set_enabled(parameter_dictionary->GetBool("enabled"));
  }
  if (parameter_dictionary->HasKey("enable_rgbd_exploration")) {
    options.set_enable_rgbd_exploration(
        parameter_dictionary->GetBool("enable_rgbd_exploration"));
  }
  if (parameter_dictionary->HasKey("enable_yopo_track")) {
    options.set_enable_yopo_track(
        parameter_dictionary->GetBool("enable_yopo_track"));
  }
  SetStringIfPresent(parameter_dictionary, "exploration_config",
                     options.mutable_exploration_config());
  SetStringIfPresent(parameter_dictionary, "odom_topic",
                     options.mutable_odom_topic());
  SetStringIfPresent(parameter_dictionary, "depth_topic",
                     options.mutable_depth_topic());
  SetStringIfPresent(parameter_dictionary, "camera_info_topic",
                     options.mutable_camera_info_topic());
  SetStringIfPresent(parameter_dictionary, "camera_frame",
                     options.mutable_camera_frame());
  SetStringIfPresent(parameter_dictionary, "map_frame",
                     options.mutable_map_frame());
  if (parameter_dictionary->HasKey("planner_hz")) {
    options.set_planner_hz(parameter_dictionary->GetDouble("planner_hz"));
  }
  SetStringIfPresent(parameter_dictionary, "path_topic",
                     options.mutable_path_topic());
  SetStringIfPresent(parameter_dictionary, "waypoint_topic",
                     options.mutable_waypoint_topic());
  SetStringIfPresent(parameter_dictionary, "map_topic",
                     options.mutable_map_topic());
  SetStringIfPresent(parameter_dictionary, "explorer_backend",
                     options.mutable_explorer_backend());
  SetStringIfPresent(parameter_dictionary, "point_cloud_topic",
                     options.mutable_point_cloud_topic());
  SetStringIfPresent(parameter_dictionary, "global_path_topic",
                     options.mutable_global_path_topic());
  SetStringIfPresent(parameter_dictionary, "local_path_topic",
                     options.mutable_local_path_topic());
  SetStringIfPresent(parameter_dictionary, "exploration_finished_topic",
                     options.mutable_exploration_finished_topic());
  SetStringIfPresent(parameter_dictionary, "exploration_progress_topic",
                     options.mutable_exploration_progress_topic());
  SetStringIfPresent(parameter_dictionary, "navigation_boundary_topic",
                     options.mutable_navigation_boundary_topic());
  SetStringIfPresent(parameter_dictionary, "terrain_map_topic",
                     options.mutable_terrain_map_topic());
  if (parameter_dictionary->HasKey("enable_exploration_nav_bridge")) {
    options.set_enable_exploration_nav_bridge(
        parameter_dictionary->GetBool("enable_exploration_nav_bridge"));
  }
  SetStringIfPresent(parameter_dictionary, "prior_map_topic",
                     options.mutable_prior_map_topic());
  SetStringIfPresent(parameter_dictionary, "vg_markers_topic",
                     options.mutable_vg_markers_topic());
  SetStringIfPresent(parameter_dictionary, "track_config",
                     options.mutable_track_config());
  SetStringIfPresent(parameter_dictionary, "cmd_vel_topic",
                     options.mutable_cmd_vel_topic());
  return options;
}

}  // namespace common
}  // namespace perception
}  // namespace autonomy
