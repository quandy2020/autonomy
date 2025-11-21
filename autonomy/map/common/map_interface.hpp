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

#include "autonomy/map/proto/map_options.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/sensor/data.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace map {
namespace common {

class MapInterface
{
public:
    /**
     * Define MapInterface::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(MapInterface)

    MapInterface() {}
    virtual ~MapInterface() {}

    MapInterface(const MapInterface&) = delete;
    MapInterface& operator=(const MapInterface&) = delete;

    /**
     * @brief  Subscribes to sensor topics if necessary and starts costmap
     * updates, can be called to restart the costmap after calls to either
     * stop() or pause()
     */
    virtual void Start() = 0;

    /**
     * @brief  Stops costmap updates and unsubscribes from sensor topics
     */
    virtual void Stop() = 0;

    /**
     * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
     */
    virtual void Pause() = 0;

    /**
     * @brief  Resumes costmap updates
     */
    virtual void Resume() = 0;

    /**
     * @brief  Adds sensor data to the map
     * @param  data The sensor data to add
     */
    void AddSensorData(std::unique_ptr<sensor::Data> data);
};

proto::MapOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);


}  // namespace common
}  // namespace map
}  // namespace autonomy