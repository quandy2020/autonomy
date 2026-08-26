/*
 * DuplicationFilter.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_filters/duplication_filter.hpp"

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

using namespace filters;

namespace grid_map {

DuplicationFilter::DuplicationFilter() = default;

DuplicationFilter::~DuplicationFilter() = default;

bool DuplicationFilter::configure() {
  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    AERROR << "DuplicationFilter did not find parameter 'input_layer'.";
    return false;
  }

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    AERROR << "DuplicationFilter did not find parameter 'output_layer'.";
    return false;
  }

  return true;
}

bool DuplicationFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  mapOut.add(outputLayer_, mapIn[inputLayer_]);
  return true;
}

}  // namespace grid_map
