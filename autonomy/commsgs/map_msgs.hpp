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

#include <vector>
#include <string>

#include "autonomy/common/port.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/proto/map_msgs.pb.h"

namespace autonomy {
namespace commsgs {
namespace map_msgs {


// An array of cells in a 2D grid
struct GridCells
{
    std_msgs::Header header;

    // Width of each cell
    float cell_width;

    // Height of each cell
    float cell_height;

    // Each cell is represented by the Point at the center of the cell
    std::vector<geometry_msgs::Point> cells;
};

// This hold basic information about the characteristics of the OccupancyGrid
struct MapMetaData
{
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

struct OccupancyGrid
{
    // This represents a 2-D grid map
    std_msgs::Header header;

    // MetaData for the map
    MapMetaData info;

    // The map data, in row-major order, starting with (0,0). 
    // Cell (1, 0) will be listed second, representing the next cell in the x direction. 
    // Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
    // The values inside are application dependent, but frequently, 
    // 0 represents unoccupied, 1 represents definitely occupied, and
    // -1 represents unknown. 
    std::vector<uint8> data;
};

struct Octomap
{
    // A 3D map in binary format, as Octree
    std_msgs::Header header;

    // Flag to denote a binary (only free/occupied) or full occupancy octree (.bt/.ot file)
    bool binary;

    // Class id of the contained octree 
    std::string id;

    // Resolution (in m) of the smallest octree nodes
    double resolution;

    // binary serialization of octree, use conversions.h to read and write octrees
    std::vector<int32> data;
};

struct OctomapWithPose
{
    // A 3D map in binary format, as Octree
    std_msgs::Header header;

    // The pose of the octree with respect to the header frame 
    geometry_msgs::Pose origin;

    // The actual octree msg
    Octomap octomap;
};

struct GridMapInfo
{
    // Header (time and frame)
    std_msgs::Header header ;

    // Resolution of the grid [m/cell].
    float resolution;

    // Length in x-direction [m].
    float length_x;

    // Length in y-direction [m].
    float length_y;

    // Pose of the grid map center in the frame defined in `header` [m].
    geometry_msgs::Pose pose;
};

struct GridMap
{
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

}  // namespace map_msgs
}  // namespace commsgs
}  // namespace autonomy