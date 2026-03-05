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

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/common/map_interface.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/proto/map_grid_option.pb.h"

namespace autonomy {
namespace map {
namespace grid_map {

class GridMapWrapper : public common::MapInterface {
 public:
  /**
   * Define GridMapWrapper::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(GridMapWrapper)

  /**
   * @brief A constructor for autonomy::map::grid_map::GridMapWrapper
   * @param options Additional options to control creation of the grid map.
   * @param name Optional name for the grid map.
   */
  GridMapWrapper(const proto::GridMapOptions& options, const std::string& name = "");

  /**
   * @brief A Destructor for autonomy::map::grid_map::GridMapWrapper
   */
  ~GridMapWrapper();

  /**
   * @brief  Subscribes to sensor topics if necessary and starts grid map
   * updates, can be called to restart the grid map after calls to either
   * stop() or pause()
   */
  void Start() override;

  /**
   * @brief  Stops grid map updates and unsubscribes from sensor topics
   */
  void Stop() override;

  /**
   * @brief  Stops the grid map from updating, but sensor data still comes in
   * over the wire
   */
  void Pause() override;

  /**
   * @brief  Resumes grid map updates
   */
  void Resume() override;

  /**
   * @brief Get the grid map object
   * @return Pointer to the GridMap
   */
  std::shared_ptr<::grid_map::GridMap> getGridMap() { return grid_map_; }

  /**
   * @brief Get const grid map object
   * @return Const pointer to the GridMap
   */
  std::shared_ptr<const ::grid_map::GridMap> getGridMap() const { return grid_map_; }

  /**
   * @brief Returns grid map name
   */
  std::string getName() const { return name_; }

  /**
   * @brief Load the map from file
   * @param filename File path to load from
   * @return Whether the map was loaded successfully
   */
  bool loadMap(const std::string& filename);

  /**
   * @brief Publish the map to the topic
   */
  void publishMap();

 protected:
  // Grid map data
  std::shared_ptr<::grid_map::GridMap> grid_map_;

  // Grid map name
  std::string name_;

  // State management
  std::atomic<bool> stopped_{true};
  std::atomic<bool> paused_{false};
  std::mutex mutex_;

  // Options for grid map
  proto::GridMapOptions options_;
};

proto::GridMapOptions CreateGridMapOptions(::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace grid_map
}  // namespace map
}  // namespace autonomy
