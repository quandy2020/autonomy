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

#include "autonomy/commsgs/map_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace map_msgs {

proto::map_msgs::GridCells ToProto(const GridCells& data)
{
    proto::map_msgs::GridCells proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_cell_width(data.cell_width);
    proto.set_cell_height(data.cell_height);
    // std::vector<geometry_msgs::Point> cells;
    return proto;
}

GridCells FromProto(const proto::map_msgs::GridCells& proto)
{
    GridCells data;
    data.header = std_msgs::FromProto(proto.header());
    data.cell_width = proto.cell_width();
    data.cell_height = proto.cell_height();
    // // Each cell is represented by the Point at the center of the cell
    // std::vector<geometry_msgs::Point> cells;
    return data;
}

proto::map_msgs::MapMetaData ToProto(const MapMetaData& data)
{
    proto::map_msgs::MapMetaData proto;
    *proto.mutable_map_load_time() = builtin_interfaces::ToProto(data.map_load_time);
    proto.set_width(data.width);
    proto.set_height(data.height);
    *proto.mutable_origin() = geometry_msgs::ToProto(data.origin);
    return proto;
}

MapMetaData FromProto(const proto::map_msgs::MapMetaData& proto)
{
    // return {
    //     builtin_interfaces::FromProto(proto.map_load_time()),
    //     proto.width(),
    //     proto.height(),
    //     geometry_msgs::FromProto(proto.origin())
    // };

    MapMetaData data;
    return data;
}

proto::map_msgs::OccupancyGrid ToProto(const OccupancyGrid& data)
{
    proto::map_msgs::OccupancyGrid proto;
    return proto;
}

OccupancyGrid FromProto(const proto::map_msgs::OccupancyGrid& proto)
{
    OccupancyGrid data;
    return data;
}

proto::map_msgs::Octomap ToProto(const Octomap& data)
{
    proto::map_msgs::Octomap proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_binary(data.binary);
    proto.set_id(data.id);
    proto.set_resolution(data.resolution);
    // std::vector<int32> data;
    return proto;
}

Octomap FromProto(const proto::map_msgs::Octomap& proto)
{
    Octomap data;
    data.header = std_msgs::FromProto(proto.header());
    data.binary = proto.binary();
    data.id = proto.id();
    data.resolution = proto.resolution();
    //  std::vector<int32> data;
    return data;
}

proto::map_msgs::OctomapWithPose ToProto(const OctomapWithPose& data)
{
    proto::map_msgs::OctomapWithPose proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_origin() = geometry_msgs::ToProto(data.origin);
    *proto.mutable_octomap() = ToProto(data.octomap);
    return proto;
}

OctomapWithPose FromProto(const proto::map_msgs::OctomapWithPose& proto)
{
    return {
        std_msgs::FromProto(proto.header()),
        geometry_msgs::FromProto(proto.origin()),
        FromProto(proto.octomap())
    };
}

}  // namespace map_msgs
}  // namespace commsgs
}  // namespace autonomy