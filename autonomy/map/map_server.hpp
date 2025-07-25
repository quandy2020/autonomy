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

#include "autonomy/common/macros.hpp"
#include "autonomy/map/common/map_interface.hpp"

#include "eventpp/eventqueue.h"

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
     * @brief A constructor for nav2_map_server::MapServer
     * @param options Additional options to control creation of the node.
     */
    explicit MapServer();

    /**
     * @brief A Destructor for nav2_map_server::MapServer
     */
    ~MapServer();

protected:

    // true if msg_ was initialized
    bool map_available_;
};

}  // namespace map
}  // namespace autonomy
