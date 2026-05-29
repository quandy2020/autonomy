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

#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/port.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/proto/map_msgs.pb.h"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace map_msgs {

// An array of cells in a 2D grid
struct GridCells {
    std_msgs::Header header;

    // Width of each cell
    float cell_width;

    // Height of each cell
    float cell_height;

    // Each cell is represented by the Point at the center of the cell
    std::vector<geometry_msgs::Point> cells;
};

// This hold basic information about the characteristics of the OccupancyGrid
struct MapMetaData {
    // The time at which the map was loaded
    builtin_interfaces::Time map_load_time;

    // The map resolution [m/cell]
    float resolution;

    // Map width [cells]
    uint32 width;

    // Map height [cells]
    uint32 height;

    // The origin of the map [m, m, rad].  This is the real-world pose of the
    // bottom left corner of cell (0,0) in the map.
    geometry_msgs::Pose origin;
};

struct OccupancyGrid {
    // Define OccupancyGrid::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(OccupancyGrid)

    // This represents a 2-D grid map
    std_msgs::Header header;

    // MetaData for the map
    MapMetaData info;

    // The map data, in row-major order, starting with (0,0).
    // Cell (1, 0) will be listed second, representing the next cell in the x
    // direction. Cell (0, 1) will be at the index equal to info.width, followed
    // by (1, 1). The values inside are application dependent, but frequently, 0
    // represents unoccupied, 1 represents definitely occupied, and -1
    // represents unknown.
    std::vector<int16> data;
};
struct OccupancyGridUpdate {
    // Define OccupancyGrid::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(OccupancyGridUpdate)

    std_msgs::Header header;
    int32 x;
    int32 y;
    uint32 width;
    uint32 height;
    std::vector<int8> data;
};
struct Octomap {
    // A 3D map in binary format, as Octree
    std_msgs::Header header;

    // Flag to denote a binary (only free/occupied) or full occupancy octree
    // (.bt/.ot file)
    bool binary;

    // Class id of the contained octree
    std::string id;

    // Resolution (in m) of the smallest octree nodes
    double resolution;

    // binary serialization of octree, use conversions.h to read and write
    // octrees
    std::vector<int32> data;
};

struct OctomapWithPose {
    // A 3D map in binary format, as Octree
    std_msgs::Header header;

    // The pose of the octree with respect to the header frame
    geometry_msgs::Pose origin;

    // The actual octree msg
    Octomap octomap;
};

struct GridMapInfo {
    // Header (time and frame)
    std_msgs::Header header;

    // Resolution of the grid [m/cell].
    float resolution;

    // Length in x-direction [m].
    float length_x;

    // Length in y-direction [m].
    float length_y;

    // Pose of the grid map center in the frame defined in `header` [m].
    geometry_msgs::Pose pose;
};

struct GridMap {
    // Grid map header
    GridMapInfo info;

    // Grid map layer names.
    std::vector<std::string> layers;

    // Grid map basic layer names (optional). The basic layers
    // determine which layers from `layers` need to be valid
    // in order for a cell of the grid map to be valid.
    std::vector<std::string> basic_layers;

    // Grid map data.
    std::vector<std_msgs::Float32MultiArray> data;

    // Row start index (default 0).
    uint32 outer_start_index;

    // Column start index (default 0).
    uint32 inner_start_index;
};

struct CostmapMetaData {
    AUTONOMY_SMART_PTR_DEFINITIONS(CostmapMetaData)

    builtin_interfaces::Time map_load_time;
    builtin_interfaces::Time update_time;
    std::string layer;
    float resolution{0.0F};
    uint32_t size_x{0};
    uint32_t size_y{0};
    geometry_msgs::Pose origin;
};

struct Costmap {
    AUTONOMY_SMART_PTR_DEFINITIONS(Costmap)

    std_msgs::Header header;
    CostmapMetaData metadata;
    std::vector<uint8_t> data;
};

struct CostmapUpdate {
    AUTONOMY_SMART_PTR_DEFINITIONS(CostmapUpdate)

    std_msgs::Header header;
    uint32_t x{0};
    uint32_t y{0};
    uint32_t size_x{0};
    uint32_t size_y{0};
    std::vector<uint8_t> data;
};

struct CostmapFilterInfo {
    AUTONOMY_SMART_PTR_DEFINITIONS(CostmapFilterInfo)

    std_msgs::Header header;
    uint32_t type{0};
    std::string filter_mask_topic;
    float base{0.0F};
    float multiplier{0.0F};
};

struct VoxelGrid {
    AUTONOMY_SMART_PTR_DEFINITIONS(VoxelGrid)

    std_msgs::Header header;
    std::vector<uint32_t> data;
    geometry_msgs::Point32 origin;
    geometry_msgs::Vector3 resolutions;
    uint32_t size_x{0};
    uint32_t size_y{0};
    uint32_t size_z{0};
};

struct GetCostmapRequest {
    CostmapMetaData specs;
};

struct GetCostmapResponse {
    Costmap map;
};

struct ClearCostmapExceptRegionRequest {
    float reset_distance{3.0F};
};

struct ClearCostmapAroundRobotRequest {
    float reset_distance{1.0F};
};

struct LoadMapRequest {
    std::string map_url;
};

struct LoadMapResponse {
    static constexpr uint8_t kResultSuccess = 0;
    static constexpr uint8_t kResultMapDoesNotExist = 1;
    static constexpr uint8_t kResultInvalidMapData = 2;
    static constexpr uint8_t kResultInvalidMapMetadata = 3;
    static constexpr uint8_t kResultUndefinedFailure = 255;

    OccupancyGrid map;
    uint8_t result{kResultUndefinedFailure};
};

struct SaveMapRequest {
    std::string map_url;
};

struct SaveMapResponse {
    uint8_t result{LoadMapResponse::kResultUndefinedFailure};
};

// Converts 'data' to a proto::GridCells.
proto::map_msgs::GridCells ToProto(const GridCells& data);

// Converts 'proto' to GridCells.
GridCells FromProto(const proto::map_msgs::GridCells& proto);

// Converts 'data' to a proto::MapMetaData.
proto::map_msgs::MapMetaData ToProto(const MapMetaData& data);

// Converts 'proto' to MapMetaData.
MapMetaData FromProto(const proto::map_msgs::MapMetaData& proto);

// Converts 'range_data' to a proto::OccupancyGrid.
proto::map_msgs::OccupancyGrid ToProto(const OccupancyGrid& data);

// Converts 'proto' to OccupancyGrid.
OccupancyGrid FromProto(const proto::map_msgs::OccupancyGrid& proto);

// Converts 'range_data' to a proto::Octomap.
proto::map_msgs::Octomap ToProto(const Octomap& data);

// Converts 'proto' to Octomap.
Octomap FromProto(const proto::map_msgs::Octomap& proto);

// Converts 'range_data' to a proto::OctomapWithPose.
proto::map_msgs::OctomapWithPose ToProto(const OctomapWithPose& data);

// Converts 'proto' to OctomapWithPose.
OctomapWithPose FromProto(const proto::map_msgs::OctomapWithPose& proto);

proto::map_msgs::CostmapFilterInfo ToProto(const CostmapFilterInfo& data);
CostmapFilterInfo FromProto(const proto::map_msgs::CostmapFilterInfo& proto);

proto::map_msgs::CostmapMetaData ToProto(const CostmapMetaData& data);
CostmapMetaData FromProto(const proto::map_msgs::CostmapMetaData& proto);

proto::map_msgs::Costmap ToProto(const Costmap& data);
Costmap FromProto(const proto::map_msgs::Costmap& proto);

}  // namespace map_msgs
}  // namespace commsgs
}  // namespace autonomy