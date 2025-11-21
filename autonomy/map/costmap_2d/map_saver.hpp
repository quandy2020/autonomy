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

#include "autonomy/map/costmap_2d/map_io.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class nav2_map_server::MapSaver
 * @brief A class that provides map saving methods and services
 */
class MapSaver
{
public:
  /**
   * @brief Constructor for the nav2_map_server::MapSaver
   * @param options Additional options to control creation of the node.
   */
  explicit MapSaver();

  /**
   * @brief Destructor for the nav2_map_server::MapServer
   */
  ~MapSaver();

  // /**
  //  * @brief Read a message from incoming map topic and save map to a file
  //  * @param map_topic Incoming map topic name
  //  * @param save_parameters Map saving parameters.
  //  * @return true of false
  //  */
  // bool saveMapTopicToFile(
  //   const std::string & map_topic,
  //   const SaveParameters & save_parameters);

protected:
  // Default values for map thresholds
  double free_thresh_default_;
  double occupied_thresh_default_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
