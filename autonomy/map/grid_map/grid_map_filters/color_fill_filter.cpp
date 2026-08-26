/*
 * ColorFillFilter.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_filters/color_fill_filter.hpp"

#include <math.h>
#include <Eigen/Dense>

#include "autonomy/map/grid_map/grid_map_core/grid_map_core.hpp"

using namespace filters;

namespace grid_map {

ColorFillFilter::ColorFillFilter() : r_(0.0), g_(0.0), b_(0.0) {}

ColorFillFilter::~ColorFillFilter() = default;

bool ColorFillFilter::configure() {
  if (!FilterBase::getParam(std::string("red"), r_)) {
    AERROR << "Color fill filter did not find parameter `red`.";
    return false;
  }
  ADEBUG << "Color fill filter red is = " << (r_) << ".";

  if (!FilterBase::getParam(std::string("green"), g_)) {
    AERROR << "Color fill filter did not find parameter `green`.";
    return false;
  }
  ADEBUG << "Color fill filter green is = " << (g_) << ".";

  if (!FilterBase::getParam(std::string("blue"), b_)) {
    AERROR << "Color fill filter did not find parameter `blue`.";
    return false;
  }
  ADEBUG << "Color fill filter blue is = " << (b_) << ".";

  if (!FilterBase::getParam(std::string("mask_layer"), maskLayer_)) {
    AWARN << "Color fill filter did not find parameter `mask_layer`.";
  } else {
    ADEBUG << "Color fill filter mask_layer = " << (maskLayer_.c_str()) << ".";
  }

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    AERROR << "Color fill filter did not find parameter `output_layer`.";
    return false;
  }
  ADEBUG << "Color fill filter output_layer = " << (outputLayer_.c_str()) << ".";
  return true;
}

bool ColorFillFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  const Eigen::Vector3f colorVector(r_, g_, b_);
  float colorValue{NAN};
  colorVectorToValue(colorVector, colorValue);

  if (maskLayer_.empty()) {
    mapOut.add(outputLayer_, colorValue);

  } else {
    mapOut.add(outputLayer_);
    auto& output = mapOut[outputLayer_];
    auto& mask = mapOut[maskLayer_];

    // For each cell in map.
    for (Eigen::Index i = 0; i < output.size(); ++i) {
      output(i) = std::isfinite(mask(i)) ? colorValue : NAN;
    }
  }
  return true;
}

}  // namespace grid_map
