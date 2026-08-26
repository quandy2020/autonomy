/*
 * MeanInRadiusFilter.cpp
 *
 *  Created on: May 3, 2017
 *      Author: Tanja Baumann, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_filters/mean_in_radius_filter.hpp"

#include <math.h>
#include "autonomy/map/grid_map/grid_map_core/grid_map_core.hpp"

using namespace filters;

namespace grid_map {

MeanInRadiusFilter::MeanInRadiusFilter() : radius_(0.0) {}

MeanInRadiusFilter::~MeanInRadiusFilter() = default;

bool MeanInRadiusFilter::configure() {
  if (!FilterBase::getParam(std::string("radius"), radius_)) {
    AERROR << "MeanInRadius filter did not find parameter `radius`.";
    return false;
  }

  if (radius_ < 0.0) {
    AERROR << "MeanInRadius filter: Radius must be greater than zero.";
    return false;
  }

  ADEBUG << "Radius = " << (radius_) << ".";

  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    AERROR << "MeanInRadius filter did not find parameter `input_layer`.";
    return false;
  }

  ADEBUG << "MeanInRadius input layer is = " << (inputLayer_.c_str()) << ".";

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    AERROR << "MeanInRadius filter did not find parameter `output_layer`.";
    return false;
  }

  ADEBUG << "MeanInRadius output_layer = " << (outputLayer_.c_str()) << ".";
  return true;
}

bool MeanInRadiusFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  // Add new layers to the elevation map.
  mapOut = mapIn;
  mapOut.add(outputLayer_);

  double value{NAN};

  // First iteration through the elevation map.
  for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    double valueSum = 0.0;
    int counter = 0;
    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Find the mean in a circle around the center
    for (grid_map::CircleIterator submapIterator(mapOut, center, radius_); !submapIterator.isPastEnd(); ++submapIterator) {
      if (!mapOut.isValid(*submapIterator, inputLayer_)) {
        continue;
      }
      value = mapOut.at(inputLayer_, *submapIterator);
      valueSum += value;
      counter++;
    }

    if (counter != 0) {
      mapOut.at(outputLayer_, *iterator) = valueSum / counter;
    }
  }

  return true;
}

}  // namespace grid_map
