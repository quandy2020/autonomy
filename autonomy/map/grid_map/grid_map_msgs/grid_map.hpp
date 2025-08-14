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

#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/proto/map_msgs.pb.h"

namespace grid_map {

using GridMapInfoData = autonomy::commsgs::map_msgs::GridMapInfo;
using GridMapInfoProto = autonomy::commsgs::proto::map_msgs::GridMapInfo;

using GridMapData = autonomy::commsgs::map_msgs::GridMap;
using GridMapProto = autonomy::commsgs::proto::map_msgs::GridMap;

// Converts 'data' to a proto::map_msgs::GridMapInfo.
GridMapInfoProto ToProto(const GridMapInfoData& data);

// Converts 'proto' to GridMapInfo.
GridMapInfoData FromProto(const GridMapInfoProto& proto);

// Converts 'data' to a proto::map_msgs::GridMap.
GridMapProto ToProto(const GridMapData& data);

// Converts 'proto' to GridMapData.
GridMapData FromProto(const GridMapProto& proto);

}  // namespace grid_map