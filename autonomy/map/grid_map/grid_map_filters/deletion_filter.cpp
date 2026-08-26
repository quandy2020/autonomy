/*
 * DeletionFilter.cpp
 *
 *  Created on: Mar 19, 2015
 *      Author: Martin Wermelinger, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_filters/deletion_filter.hpp"

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

using namespace filters;

namespace grid_map {

DeletionFilter::DeletionFilter() = default;

DeletionFilter::~DeletionFilter() = default;

bool DeletionFilter::configure() {
  // Load Parameters
  if (!FilterBase::getParam(std::string("layers"), layers_)) {
    AERROR << "DeletionFilter did not find parameter 'layers'.";
    return false;
  }

  return true;
}

bool DeletionFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;

  for (const auto& layer : layers_) {
    // Check if layer exists.
    if (!mapOut.exists(layer)) {
      AERROR << "Check your deletion layers! Type " << (layer.c_str()) << " does not exist.";
      continue;
    }

    if (!mapOut.erase(layer)) {
      AERROR << "Could not remove type " << (layer.c_str()) << ".";
    }
  }

  return true;
}

}  // namespace grid_map
