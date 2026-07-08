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

proto::map_msgs::GridCells ToProto(const GridCells& data) {
    proto::map_msgs::GridCells proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_cell_width(data.cell_width);
    proto.set_cell_height(data.cell_height);
    // std::vector<geometry_msgs::Point> cells;
    return proto;
}

GridCells FromProto(const proto::map_msgs::GridCells& proto) {
    GridCells data;
    data.header = std_msgs::FromProto(proto.header());
    data.cell_width = proto.cell_width();
    data.cell_height = proto.cell_height();
    // // Each cell is represented by the Point at the center of the cell
    // std::vector<geometry_msgs::Point> cells;
    return data;
}

proto::map_msgs::MapMetaData ToProto(const MapMetaData& data) {
    proto::map_msgs::MapMetaData proto;
    *proto.mutable_map_load_time() =
        builtin_interfaces::ToProto(data.map_load_time);
    proto.set_width(data.width);
    proto.set_height(data.height);
    *proto.mutable_origin() = geometry_msgs::ToProto(data.origin);
    return proto;
}

MapMetaData FromProto(const proto::map_msgs::MapMetaData& proto) {
    MapMetaData data;
    data.map_load_time = builtin_interfaces::FromProto(proto.map_load_time());
    data.width = proto.width();
    data.height = proto.height();
    data.origin = geometry_msgs::FromProto(proto.origin());
    return data;
}

proto::map_msgs::OccupancyGrid ToProto(const OccupancyGrid& data) {
    proto::map_msgs::OccupancyGrid proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_info() = ToProto(data.info);
    for (int16 value : data.data) {
        proto.add_data(value);
    }
    return proto;
}

OccupancyGrid FromProto(const proto::map_msgs::OccupancyGrid& proto) {
    OccupancyGrid data;
    data.header = std_msgs::FromProto(proto.header());
    data.info = FromProto(proto.info());
    data.data.reserve(proto.data_size());
    for (int32 value : proto.data()) {
        data.data.push_back(static_cast<int16>(value));
    }
    return data;
}

bool OccupancyGrid::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool OccupancyGrid::ParseFromString(const std::string& in) {
    proto::map_msgs::OccupancyGrid proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

proto::map_msgs::Octomap ToProto(const Octomap& data) {
    proto::map_msgs::Octomap proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_binary(data.binary);
    proto.set_id(data.id);
    proto.set_resolution(data.resolution);
    // std::vector<int32> data;
    return proto;
}

Octomap FromProto(const proto::map_msgs::Octomap& proto) {
    Octomap data;
    data.header = std_msgs::FromProto(proto.header());
    data.binary = proto.binary();
    data.id = proto.id();
    data.resolution = proto.resolution();
    //  std::vector<int32> data;
    return data;
}

proto::map_msgs::OctomapWithPose ToProto(const OctomapWithPose& data) {
    proto::map_msgs::OctomapWithPose proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_origin() = geometry_msgs::ToProto(data.origin);
    *proto.mutable_octomap() = ToProto(data.octomap);
    return proto;
}

OctomapWithPose FromProto(const proto::map_msgs::OctomapWithPose& proto) {
    return {std_msgs::FromProto(proto.header()),
            geometry_msgs::FromProto(proto.origin()),
            FromProto(proto.octomap())};
}

proto::map_msgs::CostmapFilterInfo ToProto(const CostmapFilterInfo& data) {
    proto::map_msgs::CostmapFilterInfo proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_type(data.type);
    proto.set_filter_mask_topic(data.filter_mask_topic);
    proto.set_base(data.base);
    proto.set_multiplier(data.multiplier);
    return proto;
}

CostmapFilterInfo FromProto(const proto::map_msgs::CostmapFilterInfo& proto) {
    CostmapFilterInfo data;
    data.header = std_msgs::FromProto(proto.header());
    data.type = proto.type();
    data.filter_mask_topic = proto.filter_mask_topic();
    data.base = proto.base();
    data.multiplier = proto.multiplier();
    return data;
}

proto::map_msgs::CostmapMetaData ToProto(const CostmapMetaData& data) {
    proto::map_msgs::CostmapMetaData proto;
    *proto.mutable_map_load_time() =
        builtin_interfaces::ToProto(data.map_load_time);
    *proto.mutable_update_time() = builtin_interfaces::ToProto(data.update_time);
    proto.set_layer(data.layer);
    proto.set_resolution(data.resolution);
    proto.set_size_x(data.size_x);
    proto.set_size_y(data.size_y);
    *proto.mutable_origin() = geometry_msgs::ToProto(data.origin);
    return proto;
}

CostmapMetaData FromProto(const proto::map_msgs::CostmapMetaData& proto) {
    CostmapMetaData data;
    data.map_load_time = builtin_interfaces::FromProto(proto.map_load_time());
    data.update_time = builtin_interfaces::FromProto(proto.update_time());
    data.layer = proto.layer();
    data.resolution = proto.resolution();
    data.size_x = proto.size_x();
    data.size_y = proto.size_y();
    data.origin = geometry_msgs::FromProto(proto.origin());
    return data;
}

proto::map_msgs::Costmap ToProto(const Costmap& data) {
    proto::map_msgs::Costmap proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_metadata() = ToProto(data.metadata);
    proto.set_data(data.data.data(), data.data.size());
    return proto;
}

Costmap FromProto(const proto::map_msgs::Costmap& proto) {
    Costmap data;
    data.header = std_msgs::FromProto(proto.header());
    data.metadata = FromProto(proto.metadata());
    data.data.assign(proto.data().begin(), proto.data().end());
    return data;
}

}  // namespace map_msgs
}  // namespace commsgs
}  // namespace autonomy