/*
 * BufferNormalizerFilter.hpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <string>

#include "autonomy/map/grid_map/grid_map_filters/filters/filter_base.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"

namespace grid_map {

/*!
 * Normalizes the buffer of a map such that it has default (zero) start index.
 */
class BufferNormalizerFilter : public filters::FilterBase<GridMap> {
 public:
  /*!
   * Constructor
   */
  BufferNormalizerFilter();

  /*!
   * Destructor.
   */
  ~BufferNormalizerFilter() override;

  /*!
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /*!
   * Normalizes the buffer of a map.
   * @param mapIn the input map before normalization.
   * @param mapOut the normalized map.
   */
  bool update(const GridMap& mapIn, GridMap& mapOut) override;
};

}  // namespace grid_map
