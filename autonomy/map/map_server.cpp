/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include "autonomy/common/logging.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_3d/costmap_3d_wrapper.hpp"

#include "autonomy/map/costmap_2d/map_io.hpp"

namespace autonomy {
namespace map {

MapServer::MapServer(const proto::MapOptions& options)
    : options_{options}
{
    if (options_.use_costmap_2d()) {
        costmap_ = std::make_shared<costmap_2d::Costmap2DWrapper>(options_.costmap2d_options());
        LOG(INFO) << "Use costmap 2D map.";
    } else if (options_.use_costmap_3d()) {
        costmap_ = std::make_shared<costmap_3d::Costmap3DWrapper>(options_.costmap3d_options());
        LOG(INFO) << "Use costmap 3D map.";
    }


    std::string yaml_file = utils::GetMapDataFilesDirectory() + options_.costmap2d_options().map_file();

    LOG(INFO) << "map_file: " << yaml_file;
}

MapServer::~MapServer()
{

}

bool MapServer::LoadMapData(const std::string& filename, commsgs::map_msgs::OccupancyGrid& map_data)
{
    if (costmap_2d::loadMapFromYaml(filename, map_data) != costmap_2d::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
        LOG(ERROR) << "Load yaml file error.";
        return false;
    }
    return true;
}

bool MapServer::LoadMapData(const std::string& filename, commsgs::map_msgs::Octomap& map_data)
{
    return true;
}

proto::MapOptions CreateMapOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::MapOptions options;
    options.set_use_costmap_2d(parameter_dictionary->GetBool("use_costmap_2d"));
    options.set_use_costmap_3d(parameter_dictionary->GetBool("use_costmap_3d"));
    *options.mutable_costmap2d_options() = 
        costmap_2d::CreateCostmap2DOptions(parameter_dictionary->GetDictionary("costmap2d").get());
    *options.mutable_costmap3d_options() = 
        costmap_3d::CreateCostmap3DOptions(parameter_dictionary->GetDictionary("costmap3d").get());
    return options;
}

}  // namespace map
}  // namespace autonomy


    