/*
 * grid_map_converter.hpp
 *
 * Conversions between grid_map::GridMap and autonomy protobuf messages
 * (replacement for grid_map_ros/GridMapRosConverter).
 */

#pragma once

#include <functional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <automsgs/msgs/map_msgs/grid_map.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_sdf/signed_distance_field.hpp"

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

  static void toPointCloud(
      const GridMap& gridMap, const std::string& pointLayer,
      automsgs::msgs::sensor_msgs::PointCloud2& pointCloud);

  static void toPointCloud(
      const GridMap& gridMap, const std::vector<std::string>& layers,
      const std::string& pointLayer,
      automsgs::msgs::sensor_msgs::PointCloud2& pointCloud);

  static void toPointCloud(
      const SignedDistanceField& signedDistanceField,
      automsgs::msgs::sensor_msgs::PointCloud2& pointCloud,
      size_t decimation = 1,
      const std::function<bool(float)>& condition = [](float) { return true; });

  static bool initializeFromImage(
      const automsgs::msgs::sensor_msgs::Image& image, double resolution,
      GridMap& gridMap,
      const Position& position = Position::Zero());

  static bool addLayerFromImage(
      const automsgs::msgs::sensor_msgs::Image& image, const std::string& layer,
      GridMap& gridMap, float lowerValue = 0.0f, float upperValue = 1.0f,
      double alphaThreshold = 0.5);

  static bool addColorLayerFromImage(
      const automsgs::msgs::sensor_msgs::Image& image, const std::string& layer,
      GridMap& gridMap);

  static bool toImage(const GridMap& gridMap, const std::string& layer,
                      const std::string& encoding,
                      automsgs::msgs::sensor_msgs::Image& image);

  static bool toImage(const GridMap& gridMap, const std::string& layer,
                      const std::string& encoding, float lowerValue,
                      float upperValue,
                      automsgs::msgs::sensor_msgs::Image& image);

  static bool toCvImage(const GridMap& gridMap, const std::string& layer,
                        const std::string& encoding, cv::Mat& image);

  static bool toCvImage(const GridMap& gridMap, const std::string& layer,
                        const std::string& encoding, float lowerValue,
                        float upperValue, cv::Mat& image);

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
