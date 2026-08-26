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

#pragma once

#include <automsgs/msgs/map_msgs/grid_map.pb.h>
#include <automsgs/msgs/map_msgs/grid_map_info.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>

#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/msg_helpers.hpp"

namespace grid_map {

using GridMapInfoProto = automsgs::msgs::map_msgs::GridMapInfo;
using GridMapProto = automsgs::msgs::map_msgs::GridMap;
using OccupancyGridProto = automsgs::msgs::map_msgs::OccupancyGrid;

}  // namespace grid_map
