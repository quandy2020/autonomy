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

#pragma once 

#include <string>
#include <memory>
#include <functional>

#include "autonomy/map/proto/map_options.pb.h"
#include "autonomy/common/macros.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/common/map_interface.hpp"
#include "autonomy/map/utils/data_loader_utils.hpp"

namespace autonomy {
namespace map {
    
/**
 * @class autonomy::map::MapServer
 * @brief Parses the map yaml file and creates a service and a publisher that
 * provides occupancy grid
 */
class MapServer : public common::MapInterface
{
public:
    /**
     * Define MapServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(MapServer)

    /**
     * @brief A constructor for map::MapServer
     * @param options Additional options to control creation of the node.
     */
    MapServer(const proto::MapOptions& options);

    /**
     * @brief A Destructor for map::MapServer
     */
    ~MapServer();

    /**
     * @brief Load OccupancyGrid map data 
     * 
     * @param filename 
     * @param map_data 
     * @return true 
     * @return false 
     */
    bool LoadMapData(const std::string& filename, commsgs::map_msgs::OccupancyGrid& map_data);

    /**
     * @brief Load Octomap map data 
     * 
     * @param filename 
     * @param map_data 
     * @return true 
     * @return false 
     */
    bool LoadMapData(const std::string& filename, commsgs::map_msgs::Octomap& map_data);

    /**
     * @brief Load PointCloud map data 
     * 
     * @param filename 
     * @param map_data 
     * @return true 
     * @return false 
     */
    bool LoadMapData(const std::string& filename, commsgs::sensor_msgs::PointCloud& map_data);

    /**
     * @brief Load PointCloud map data 
     * 
     * @param filename 
     * @param map_data 
     * @return true 
     * @return false 
     */
    bool LoadMapData(const std::string& filename, commsgs::sensor_msgs::PointCloud2& map_data);

    /**
     * @brief Get OccupancyGrid map data 2d format
     * 
     * @return commsgs::map_msgs::OccupancyGrid* 
     */
    commsgs::map_msgs::OccupancyGrid* occupancy_grid_map_data() { return occupancy_grid_map_data_.get(); }

protected:
    // Costmap 2D or 3D
    common::MapInterface::SharedPtr costmap_{nullptr};

    // OccupancyGrid map data
    commsgs::map_msgs::OccupancyGrid::SharedPtr occupancy_grid_map_data_{nullptr};

    // true if msg_ was initialized
    bool map_available_;

    // Map configuration options
    proto::MapOptions options_;
};

proto::MapOptions CreateMapOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace map
}  // namespace autonomy
