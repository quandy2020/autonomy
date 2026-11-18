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
 * @file headland_generator_base.hpp
 * @brief Fields2Cover headland generator base.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types.hpp"

namespace autonomy::planning::planner::fields2cover::hg {

/**
 * @class autonomy::planning::planner::fields2cover::hg::HeadlandGeneratorBase
 * @brief Base class to Generate the headlands.
 */
class HeadlandGeneratorBase {
 public:
  /**
   * @brief Get the regions of a cell without the headlands the robot needs to turn.
   * @param[in] field Borders of the cell and the obstacles on it.
   * @param[in] robot Robot doing the coverage.
   * @param[in] track_ang Angle of the swaths on the cell.
   * @return Mainland Area
   */
  virtual F2CCells GenerateHeadlands(
    const F2CCell& field, const F2CRobot& robot, double track_ang);

  /**
   * @brief Get the regions of the field without the headlands the robot needs.
   * @param[in] field Borders of the field and the obstacles on it.
   * @param[in] robot Robot doing the coverage.
   * @param[in] track_angs Angle of the swaths on each cell.
   * @return Mainland Area
   */
  virtual F2CCells GenerateHeadlands(
    const F2CCells& field, const F2CRobot& robot,
    const std::vector<double>& track_angs);

  /**
   * @brief Width of the headland a robot can need on a border, at worst.
   */
  double MaxHLWidthRequired(const F2CRobot& robot) const;

  /**
   * @brief Get the regions of the field without headlands.
   * @param[in] _field Original field
   * @param[in] _dist_headland Width of the headland
   * @return Field without headlands
   */
  virtual F2CCells GenerateHeadlands(
    const F2CCells& field, double dist_headland) = 0;

  /**
   * @brief Generate headland Area of the field for a given number of swaths
   *   with given width.
   *
   * @param[in] field Borders of the field and the obstacles on it.
   * @param[in] swath_width Width of each headland swath.
   * @param[in] n_swaths Number of headland swaths.
   * @return Headland Area
   */
  virtual F2CCells GenerateHeadlandArea(
    const F2CCells& field, double swath_width, int n_swaths) = 0;

  /**
   * @brief Generate headland swaths of the field for a given number of swaths
   *   with given width.
   *
   * @param[in] field Borders of the field and the obstacles on it.
   * @param[in] swath_width Width of each headland swath.
   * @param[in] n_swaths Number of headland swaths.
   * @param[in] dir_out2in When true, headland swaths are created from
   *          outer borders to inner borders.
   *          Otherwise, they are created from inner borders to outers.
   * @return Vector of size \a n_swaths for each headland swath.
   *   Each F2CLinearRing on it is a headland swath ring.
   */
  virtual std::vector<F2CCells> GenerateHeadlandSwaths(
    const F2CCells& field, double swath_width, int n_swaths,
    bool dir_out2in = true) = 0;

  virtual ~HeadlandGeneratorBase() = default;
};

}  // namespace autonomy::planning::planner::fields2cover::hg

