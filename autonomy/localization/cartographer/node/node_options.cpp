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

#include "autonomy/localization/cartographer/node/node_options.hpp"

#include <memory>
#include <vector>

#include <glog/logging.h>

#include "autonomy/localization/cartographer/common/configuration_file_resolver.hpp"
#include "autonomy/localization/cartographer/mapping/map_builder_interface.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
    NodeOptions options;
    options.map_builder_options =
        ::cartographer::mapping::CreateMapBuilderOptions(
            lua_parameter_dictionary->GetDictionary("map_builder").get());
    options.map_frame = lua_parameter_dictionary->GetString("map_frame");
    options.lookup_transform_timeout_sec =
        lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
    options.submap_publish_period_sec =
        lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
    options.pose_publish_period_sec =
        lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
    options.trajectory_publish_period_sec =
        lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
    if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
        options.publish_to_tf =
            lua_parameter_dictionary->GetBool("publish_to_tf");
    }
    if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
        options.publish_tracked_pose =
            lua_parameter_dictionary->GetBool("publish_tracked_pose");
    }
    if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
        options.use_pose_extrapolator =
            lua_parameter_dictionary->GetBool("use_pose_extrapolator");
    }
    if (lua_parameter_dictionary->HasKey("publish_occupancy_grid")) {
        options.publish_occupancy_grid =
            lua_parameter_dictionary->GetBool("publish_occupancy_grid");
    }
    if (lua_parameter_dictionary->HasKey("occupancy_grid_publish_period_sec")) {
        options.occupancy_grid_publish_period_sec =
            lua_parameter_dictionary->GetDouble(
                "occupancy_grid_publish_period_sec");
    }
    if (lua_parameter_dictionary->HasKey("occupancy_grid_resolution")) {
        options.occupancy_grid_resolution =
            lua_parameter_dictionary->GetDouble("occupancy_grid_resolution");
    }
    if (lua_parameter_dictionary->HasKey("save_map_image")) {
        options.save_map_image =
            lua_parameter_dictionary->GetBool("save_map_image");
    }
    if (lua_parameter_dictionary->HasKey("map_image_save_period_sec")) {
        options.map_image_save_period_sec =
            lua_parameter_dictionary->GetDouble("map_image_save_period_sec");
    }
    if (lua_parameter_dictionary->HasKey("map_image_save_directory")) {
        options.map_image_save_directory =
            lua_parameter_dictionary->GetString("map_image_save_directory");
    }
    if (lua_parameter_dictionary->HasKey("map_image_filestem")) {
        options.map_image_filestem =
            lua_parameter_dictionary->GetString("map_image_filestem");
    }
    return options;
}

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
    auto file_resolver = std::make_unique<
        ::cartographer::common::ConfigurationFileResolver>(
        std::vector<std::string>{configuration_directory});
    const std::string code =
        file_resolver->GetFileContentOrDie(configuration_basename);
    ::cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
        code, std::move(file_resolver));

    return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                           CreateTrajectoryOptions(&lua_parameter_dictionary));
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
