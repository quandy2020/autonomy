/*
 * SetBasicLayersFilters.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_filters/set_basic_layers_filter.hpp"

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

using namespace filters;

namespace grid_map {

SetBasicLayersFilter::SetBasicLayersFilter() = default;

SetBasicLayersFilter::~SetBasicLayersFilter() = default;

bool SetBasicLayersFilter::configure() {
  if (!FilterBase::getParam(std::string("layers"), layers_)) {
    AERROR << "SetBasicLayersFilters did not find parameter 'layers'.";
    return false;
  }

  return true;
}

bool SetBasicLayersFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  std::vector<std::string> layersChecked;

  for (const auto& layer : layers_) {
    if (!mapOut.exists(layer)) {
      AWARN << "Layer `" << (layer.c_str()) << "` does not exist and is not set as basic layer.";
      continue;
    }
    layersChecked.push_back(layer);
  }

  mapOut.setBasicLayers(layersChecked);
  return true;
}

}  // namespace grid_map
