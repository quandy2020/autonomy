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

#pragma once 

#include <string>
#include <memory>
#include <functional>

#include "autonomy/map/proto/map_options.pb.h"
#include "autonomy/common/macros.hpp"
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
class MapServer 
{
public:
    /**
     * Define MapServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(MapServer)

    /**
     * @brief A constructor for map::MapServer
     * @param node The node to be used for creating the publisher and service.
     * @param options Additional options to control creation of the node.
     */
    MapServer(const proto::MapOptions& options);

    /**
     * @brief Starts server
     */
    void Start();

    /**
     * @brief Shutdown 
     */
    void WaitForShutdown();

protected:

    // Costmap 2D or 3D
    common::MapInterface::SharedPtr costmap_{nullptr};

    // Map configuration options
    proto::MapOptions options_;

};

}  // namespace map
}  // namespace autonomy
