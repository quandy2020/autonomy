/*
 * grid_map_converter.hpp
 *
 * Conversions between grid_map::GridMap and autonomy protobuf messages
 * (replacement for grid_map_ros/GridMapRosConverter).
 */

#pragma once

#include <string>
#include <vector>

#include <automsgs/msgs/map_msgs/grid_map.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

namespace grid_map {

class GridMapConverter {
 public:
  GridMapConverter() = default;
  ~GridMapConverter() = default;

  static bool fromMessage(const automsgs::msgs::map_msgs::GridMap& message,
                          GridMap& gridMap,
                          const std::vector<std::string>& layers,
                          bool copyBasicLayers = true,
                          bool copyAllNonBasicLayers = true);

  static bool fromMessage(const automsgs::msgs::map_msgs::GridMap& message,
                          GridMap& gridMap);

  static void toMessage(const GridMap& gridMap,
                        automsgs::msgs::map_msgs::GridMap& message);

  static void toMessage(const GridMap& gridMap,
                        const std::vector<std::string>& layers,
                        automsgs::msgs::map_msgs::GridMap& message);

  static bool fromOccupancyGrid(
      const automsgs::msgs::map_msgs::OccupancyGrid& occupancyGrid,
      const std::string& layer, GridMap& gridMap);

  static void toOccupancyGrid(
      const GridMap& gridMap, const std::string& layer, float dataMin,
      float dataMax, automsgs::msgs::map_msgs::OccupancyGrid& occupancyGrid);

  /*!
   * Serialize grid map to a binary protobuf file.
   */
  static bool saveToFile(const GridMap& gridMap, const std::string& filename,
                         const std::vector<std::string>& layers = {});

  /*!
   * Load grid map from a binary protobuf file previously written by saveToFile.
   */
  static bool loadFromFile(const std::string& filename, GridMap& gridMap);
};

}  // namespace grid_map
