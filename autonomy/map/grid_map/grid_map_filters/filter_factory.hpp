/*
 * filter_factory.hpp
 *
 * Create grid_map filters by type string (ROS-free replacement for pluginlib).
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_filters/filters/filter_base.hpp"

namespace grid_map {

class FilterFactory {
 public:
  /*!
   * Create a filter instance for a type string.
   * Accepts short names ("ThresholdFilter") and plugin-style names
   * ("grid_map/ThresholdFilter").
   * @return nullptr if the type is unknown.
   */
  static std::unique_ptr<filters::FilterBase<GridMap>> create(
      const std::string& type);
};

}  // namespace grid_map
