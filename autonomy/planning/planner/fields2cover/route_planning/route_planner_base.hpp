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
 * @file route_planner_base.hpp
 * @brief Fields2Cover route planner base.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include <map>
#include <optional>
#include <limits>
#include <utility>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/rp_obj/rp_objective.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/rp_obj/direct_dist_path_obj.hpp"
#include "autonomy/planning/planner/fields2cover/route_planning/single_cell_swaths_order_base.hpp"

namespace autonomy::planning::planner::fields2cover::rp {

/**
 * @class autonomy::planning::planner::fields2cover::rp::RoutePlannerBase
 * @brief Build a coverage route that visits swaths and connects them on the headland.
 */
class RoutePlannerBase {
 public:
  /**
   * @brief Generate route to cover the swaths on a field.
   *   If two consecutive swaths are far away,
   *   the route connects both through the headland.
   *
   * @param[in] cells Headland swath rings used to travel through the headlands
   * @param[in] swaths_by_cells Swaths to be covered.
   * @param[in] show_log Show log from the optimizer
   * @param[in] d_tol Tolerance Distance to consider if two points are the same.
   * @param[in] redirect_swaths Whether to allow redirecting swaths
   * @param[in] time_limit_seconds Maximum time to spend on optimization
   * @param[in] search_for_optimum If true, uses guided local search which may take longer
   *        but can find more optimal solutions. If false, uses automatic search which is faster
   *        but may find less optimal solutions.
   * @return Route that covers all the swaths
   */
  virtual F2CRoute GenRoute(
      const F2CCells& cells, const F2CSwathsByCells& swaths_by_cells,
      bool show_log = false, double d_tol = 1e-4, bool redirect_swaths = true,
      long int time_limit_seconds = 1, bool search_for_optimum = false);

  /**
   * @brief Set the start and the end of the route.
   */
  void SetStartAndEndPoint(const F2CPoint& p);

  /**
   * @brief Create graph to compute the shortest path between two points
   *   in the headlands.
   *
   * @param[in] cells Headland swath rings used to travel through the headlands
   * @param[in] swaths_by_cells Swaths to be covered.
   * @param[in] d_tol Tolerance Distance to consider if two points are the same.
   */
  virtual F2CGraph2D CreateShortestGraph(
      const F2CCells& cells, const F2CSwathsByCells& swaths_by_cells,
      double d_tol) const;

  /**
   * @brief Create graph to compute the cost of covering the swaths in a given order.
   *
   * @param[in] cells Headland swath rings used to travel through the headlands
   * @param[in] swaths_by_cells Swaths to be covered.
   * @param[in,out] shortest_graph Graph used to compute the shortest path
   *          between two nodes.
   * @param[in] d_tol Tolerance Distance to consider if two points are the same.
   * @return Coverage graph whose nodes are swath endpoints.
   */
  virtual F2CGraph2D CreateCoverageGraph(
      const F2CCells& cells, const F2CSwathsByCells& swaths_by_cells,
      F2CGraph2D& shortest_graph,
      double d_tol, bool redirect_swaths = true) const;


  virtual ~RoutePlannerBase() = default;

  /**
   * @brief Use the optimizer to Generate the index of the points of the best
   *   coverage route.
   * @param[in] cov_graph Graph representing the coverage problem
   * @param[in] show_log Whether to show optimization logs
   * @param[in] time_limit_seconds Maximum time to spend on optimization
   * @param[in] use_guided_local_search If true, uses guided local search which may take longer
   *        but can find more optimal solutions. If false, uses automatic search which is faster
   *        but may find less optimal solutions.
   */
  virtual std::vector<long long int> ComputeBestRoute(
      const F2CGraph2D& cov_graph, bool show_log, long int time_limit_seconds,
      bool use_guided_local_search = true) const;

  /**
   * @brief Tranform index of points to an actual Route.
   */
  virtual F2CRoute TransformSolutionToRoute(
      const std::vector<long long int>& route_ids,
      const F2CSwathsByCells& swaths_by_cells,
      const F2CGraph2D& coverage_graph,
      F2CGraph2D& shortest_graph) const;

 protected:
  std::optional<F2CPoint> r_start_end;
};



}  // namespace autonomy::planning::planner::fields2cover::rp

