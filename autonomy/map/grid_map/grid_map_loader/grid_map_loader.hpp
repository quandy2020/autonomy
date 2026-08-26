/*
 * GridMapLoader.hpp
 *
 * Loads a grid map from a protobuf file (replacement for ROS bag loader).
 */

#pragma once

#include <string>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

namespace grid_map_loader {

class GridMapLoader {
 public:
  GridMapLoader() = default;
  explicit GridMapLoader(std::string filePath) : filePath_(std::move(filePath)) {}
  ~GridMapLoader() = default;

  void setFilePath(const std::string& filePath) { filePath_ = filePath; }
  const std::string& filePath() const { return filePath_; }

  /*!
   * Loads the grid map from the configured protobuf file.
   * @return true if successful, false otherwise.
   */
  bool load();

  const grid_map::GridMap& getGridMap() const { return map_; }
  grid_map::GridMap& getGridMap() { return map_; }

 private:
  grid_map::GridMap map_;
  std::string filePath_;
};

}  // namespace grid_map_loader
