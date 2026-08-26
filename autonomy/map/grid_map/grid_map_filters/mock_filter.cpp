/*
 * MockFilter.cpp
 *
 *  Created on: Sep 24, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_filters/mock_filter.hpp"

#include <chrono>
#include <thread>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

using namespace filters;

namespace grid_map {

MockFilter::MockFilter() = default;

MockFilter::~MockFilter() = default;

bool MockFilter::configure() {
  if (!FilterBase::getParam(std::string("processing_time"), processingTime_)) {
    AERROR << "MockFilter did not find parameter 'processing_time'.";
    return false;
  }

  if (!FilterBase::getParam(std::string("print_name"), printName_)) {
    AINFO << "MockFilter did not find parameter 'print_name'. Not printing the name. ";
    printName_ = false;
  }

  return true;
}

bool MockFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  if (printName_) {
    AINFO << this->getName() << ": update()";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(processingTime_));
  return true;
}

}  // namespace grid_map
