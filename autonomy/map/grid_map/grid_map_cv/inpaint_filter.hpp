/*
 * InpaintFilter.hpp
 *
 *  Created on: May 6, 2017
 *      Author: Tanja Baumann, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 *
 * Adapted for autonomy (ROS-free FilterBase).
 */

#pragma once

#include <string>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_filters/filters/filter_base.hpp"

namespace grid_map {

/*!
 * Uses OpenCV inpaint to fill holes in the input layer.
 */
class InpaintFilter : public filters::FilterBase<GridMap> {
 public:
  InpaintFilter();
  ~InpaintFilter() override;

  bool update(const GridMap& mapIn, GridMap& mapOut) override;

  using filters::FilterBase<GridMap>::configure;

 protected:
  bool configure() override;

 private:
  double radius_;
  std::string inputLayer_;
  std::string outputLayer_;
};

}  // namespace grid_map
