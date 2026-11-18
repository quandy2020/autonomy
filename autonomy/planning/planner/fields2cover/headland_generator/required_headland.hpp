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
 * @file required_headland.hpp
 * @brief Fields2Cover required headland.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/headland_generator/headland_generator_base.hpp"

namespace autonomy::planning::planner::fields2cover::hg {

/**
 * @class autonomy::planning::planner::fields2cover::hg::ReqHL
 * @brief Class to Generate headlands as wide as each border needs.
 *
 * A border the swaths run along is only entered, while a border they end on
 * takes a whole turn. Sizing each one on its own leaves the difference to the
 * mainland instead of giving it up on every border.
 */
class ReqHL : public HeadlandGeneratorBase {
 public:
  /**
   * @brief Generate the mainland of a cell covered at a given track angle.
   * @param[in] field Borders of the cell and the obstacles on it.
   * @param[in] robot Robot doing the coverage.
   * @param[in] track_ang Angle of the swaths on the cell.
   * @return Mainland Area
   */
  F2CCells GenerateHeadlands(
    const F2CCell& field, const F2CRobot& robot, double track_ang) override;

  /**
   * @brief Generate the mainland of each cell covered at its own track angle.
   * @param[in] field Borders of the field and the obstacles on it.
   * @param[in] robot Robot doing the coverage.
   * @param[in] track_angs Angle of the swaths on each cell. Without one angle
   *                   per cell, every border keeps the widest headland.
   * @return Mainland Area
   */
  F2CCells GenerateHeadlands(
    const F2CCells& field, const F2CRobot& robot,
    const std::vector<double>& track_angs) override;

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
   * @param[in] field Borders of the field and the obstacles on it.
   * @param[in] swath_width Width of each headland swath.
   * @param[in] n_swaths Number of headland swaths.
   * @param[in] dir_out2in When true, headland swaths are created from
   *          outer borders to inner borders.
   *          Otherwise, they are created from inner borders to outers.
   * @return Vector of size \a n_swaths for each headland swath.
   */
  std::vector<F2CCells> GenerateHeadlandSwaths(
    const F2CCells& field, double swath_width, int n_swaths,
    bool dir_out2in = true) override;

  /**
   * @brief Width of the headland a border needs.
   * @param[in] robot Robot doing the coverage.
   * @param[in] track_ang Angle of the swaths on the cell.
   * @param[in] border_ang Angle of the border.
   * @return Distance between the border and the mainland
   */
  double RequiredHeadlandDist(const F2CRobot& robot,
      double track_ang, double border_ang) const;

  /**
   * @brief Width of the headland each segment of a ring needs.
   * @param[in] robot Robot doing the coverage.
   * @param[in] track_ang Angle of the swaths on the cell.
   * @param[in] ring Border of the cell or of an obstacle on it.
   * @return Distance for each segment of \a ring
   */
  std::vector<double> RequiredHeadlandDist(const F2CRobot& robot,
      double track_ang, const F2CLinearRing& ring) const;
};

}  // namespace autonomy::planning::planner::fields2cover::hg


