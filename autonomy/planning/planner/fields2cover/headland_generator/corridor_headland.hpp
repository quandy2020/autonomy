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
 * @file corridor_headland.hpp
 * @brief Fields2Cover corridor headland.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/headland_generator/headland_generator_base.hpp"

namespace autonomy::planning::planner::fields2cover::hg {

/**
 * @class autonomy::planning::planner::fields2cover::hg::CorridorShareMode
 * @brief How CorridorHL splits the corridor between two cells that share a border.
 */
enum class CorridorShareMode {
  /**
   * @brief The smaller cell gives the whole corridor; ties split evenly. Default.
   */
  ASYMMETRIC,
  /**
   * @brief Every shared border splits evenly, regardless of the cells' size.
   */
  SYMMETRIC,
};

/**
 * @class autonomy::planning::planner::fields2cover::hg::CorridorShare
 * @brief One shared border between two cells, and how the corridor over it is split.
 *
 * A pair of touching cells gives two of these, one seen from each cell. The
 * pair that shares the border evenly gives 0.5 twice; otherwise the smaller
 * cell gives the whole corridor and the larger one keeps a row with a share
 * of 0, so a report can tell which side of the border the corridor came from.
 */
struct CorridorShare {
  /**
   * @brief Cell the corridor is taken out of.
   */
  size_t cell_i {0};
  /**
   * @brief Neighbour on the other side of the border.
   */
  size_t cell_k {0};
  /**
   * @brief Perimeter of cell \a cell_i, the size the rule compares.
   */
  double perimeter_i {0.0};
  /**
   * @brief Perimeter of cell \a cell_k.
   */
  double perimeter_k {0.0};
  /**
   * @brief True when both perimeters are equal up to the tolerance of the rule.
   */
  bool same_size {false};
  /**
   * @brief Part of the corridor width taken out of \a cell_i: 0, 0.5 or 1.
   */
  double share {0.0};
  /**
   * @brief Length of the border the two cells share.
   */
  double shared_length {0.0};
  /**
   * @brief Border segments the corridor is opened along.
   */
  F2CMultiLineString shared_border;
};

/**
 * @class autonomy::planning::planner::fields2cover::hg::CorridorHL
 * @brief Class to open a corridor where cells border each other, leaving the edges
 * that face the outer boundary or a void untouched.
 *
 * Meant for cells that came out of a decomposition and already have a
 * headland around the field: shrinking every border again would take a second
 * headland off ground that has one.
 */
class CorridorHL : public HeadlandGeneratorBase {
 public:
  using HeadlandGeneratorBase::GenerateHeadlands;

  /**
   * @brief Open a corridor of the given width between cells that share a border.
   * @param[in] field Cells that share borders, usually from a decomposition.
   * @param[in] dist_headland Width of the corridor.
   * @return Mainland Area
   */
  F2CCells GenerateHeadlands(
    const F2CCells& field, double dist_headland) override;

  /**
   * @brief Split the corridor between each pair of cells that share a border.
   *
   * This is the rule GenerateHeadlands() applies, on its own: which cell
   * gives the corridor, how much of it, and over which part of the border.
   * @param[in] field Cells that share borders, usually from a decomposition.
   * @param[in] mode Rule to split the corridor with.
   * @return One share per ordered pair of cells that touch along a border.
   */
  std::vector<CorridorShare> CorridorShares(
      const F2CCells& field,
      CorridorShareMode mode = CorridorShareMode::ASYMMETRIC) const;

  /**
   * @brief Rule GenerateHeadlands() uses to split the corridor.
   *
   * Defaults to ASYMMETRIC.
   */
  void SetShareMode(CorridorShareMode mode);

  /**
   * @brief Rule GenerateHeadlands() currently uses to split the corridor.
   */
  CorridorShareMode GetShareMode() const;

  /**
   * @brief Open a corridor wide enough for \a n_swaths passes.
   * @param[in] field Borders of the field and the obstacles on it.
   * @param[in] swath_width Width of each headland swath.
   * @param[in] n_swaths Number of headland swaths.
   * @return Mainland Area
   */
  F2CCells GenerateHeadlandArea(
    const F2CCells& field, double swath_width, int n_swaths) override;

  /**
   * @brief Widen the corridor one swath at a time.
   * @param[in] field Borders of the field and the obstacles on it.
   * @param[in] swath_width Width of each headland swath.
   * @param[in] n_swaths Number of headland swaths.
   * @param[in] dir_out2in When true, the widest corridor comes first.
   * @return Vector of size \a n_swaths, each one carved a swath wider.
   */
  std::vector<F2CCells> GenerateHeadlandSwaths(
    const F2CCells& field, double swath_width, int n_swaths,
    bool dir_out2in = true) override;

 private:
  CorridorShareMode share_mode_ {CorridorShareMode::ASYMMETRIC};

  /**
   * @brief Tolerance the neighbour is buffered by to find the shared border.
   */
  double tol_ {1e-3};
  /**
   * @brief Width the zero-width spur a difference can leave behind is opened by.
   */
  double spur_ {1e-9};
  /**
   * @brief Tolerance two perimeters are compared with to count as the same size.
   */
  double same_size_tol_ {1e-9};
  /**
   * @brief Shortest border kept as real: buffering the neighbour by tol_ turns a
   * shared corner into a piece a few millimetres long on each edge that
   * reaches it, and a border that short is a corner, not a corridor.
   */
  double min_border_ {1e-2};
};

}  // namespace autonomy::planning::planner::fields2cover::hg


