/*
 * Copyright 2016 The Cartographer Authors
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

#pragma once

#include <string>
#include <tuple>

#include "autonomy/localization/cartographer/common/lua_parameter_dictionary.hpp"
#include "autonomy/localization/cartographer/mapping/proto/map_builder_options.pb.h"
#include "autonomy/localization/cartographer/node/trajectory_options.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

struct NodeOptions {
    ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
    std::string map_frame;
    double lookup_transform_timeout_sec;
    double submap_publish_period_sec;
    double pose_publish_period_sec;
    double trajectory_publish_period_sec;
    double occupancy_grid_publish_period_sec = 1.0;
    double occupancy_grid_resolution = 0.05;
    bool publish_to_tf = true;
    bool publish_tracked_pose = false;
    bool publish_occupancy_grid = true;
    bool save_map_image = true;
    double map_image_save_period_sec = 10.0;
    std::string map_image_save_directory = "data";
    std::string map_image_filestem = "map";
    bool use_pose_extrapolator = true;
};

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
