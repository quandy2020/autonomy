/*
 * BufferNormalizerFilter.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "autonomy/map/grid_map/grid_map_filters/buffer_normalizer_filter.hpp"

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

using namespace filters;

namespace grid_map {

BufferNormalizerFilter::BufferNormalizerFilter() = default;

BufferNormalizerFilter::~BufferNormalizerFilter() = default;

bool BufferNormalizerFilter::configure() {
  return true;
}

bool BufferNormalizerFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  mapOut.convertToDefaultStartIndex();

  return true;
}

}  // namespace grid_map
