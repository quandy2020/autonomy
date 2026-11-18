/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file constant_headland.hpp
 * @brief Fields2Cover constant headland.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/headland_generator/headland_generator_base.hpp"

namespace autonomy::planning::planner::fields2cover::hg {

/**
 * @class autonomy::planning::planner::fields2cover::hg::ConstHL
 * @brief Class to Generate headlands with equal width in each border.
 */
class ConstHL : public HeadlandGeneratorBase {
 public:
  using HeadlandGeneratorBase::GenerateHeadlands;

  /**
   * @brief Generate headland Area of the field at a given Distance from borders.
   * @param[in] field Borders of the field and the obstacles on it.
   * @param[in] dist_headland Distance between exterior and interior borders
   *                      of the headlands.
   * @return Mainland Area
   */
  F2CCells GenerateHeadlands(
    const F2CCells& field, double dist_headland) override;

  /**
   * @brief Generate headland Area of the field for a given number of swaths
   *   with given width.
   * @param[in] field Borders of the field and the obstacles on it.
   * @param[in] swath_width Width of each headland swath.
   * @param[in] n_swaths Number of headland swaths.
   * @return Headland Area
   */
  F2CCells GenerateHeadlandArea(
    const F2CCells& field, double swath_width, int n_swaths) override;

  /**
   * @brief Generate headland swaths of the field for a given number of swaths
   * with given width.
   *
   * @param[in] field Borders of the field and the obstacles on it.
   * @param[in] swath_width Width of each headland swath.
   * @param[in] n_swaths Number of headland swaths.
   * @param[in] dir_out2in When true, headland swaths are created from
   *          outer borders to inner borders.
   *           Otherwise, they are created from inner borders to outers.
   * @return Vector of size \a n_swaths for each headland swath.
   *   Each F2CLinearRing on it is a headland swath ring.
   */
  std::vector<F2CCells> GenerateHeadlandSwaths(
    const F2CCells& field, double swath_width, int n_swaths,
    bool dir_out2in = true) override;
};

}  // namespace autonomy::planning::planner::fields2cover::hg


