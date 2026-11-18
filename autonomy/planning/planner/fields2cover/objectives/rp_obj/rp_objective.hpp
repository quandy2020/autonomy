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
 * @file rp_objective.hpp
 * @brief Fields2Cover rp objective.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/base_objective.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

/**
 * @class autonomy::planning::planner::fields2cover::obj::RPObjective
 * @brief Base class for objective functions of route planners.
 */
class RPObjective : public BaseObjective<RPObjective> {
 public:
  /**
   * @brief Return the cost of going from point p1 to point p2.
   * @param[in] p1 Start point
   * @param[in] p2 End point
   * @return Cost value
   */
  virtual double ComputeCost(const F2CPoint& p1, const F2CPoint& p2);

  /**
   * @brief Return the cost of going from point p1 to point p2.
   * @param[in] p1 Start point
   * @param[in] ang1 Angle of the robot in p1
   * @param[in] p2 End point
   * @return Cost value
   */
  virtual double ComputeCost(
      const F2CPoint& p1, double ang1, const F2CPoint& p2);

  /**
   * @brief Return the cost of going from point p1 to point p2.
   * @param[in] p1 Start point
   * @param[in] ang1 Angle of the robot in p1
   * @param[in] p2 End point
   * @param[in] ang2 Angle of the robot in p2
   * @return Cost value
   */
  virtual double ComputeCost(
      const F2CPoint& p1, double ang1, const F2CPoint& p2, double ang2);  // = 0

  /**
   * @brief Return the cost of going from point p1 to point p2.
   * @param[in] p1 Start point
   * @param[in] p2 End point
   * @param[in] ang2 Angle of the robot in p2
   * @return Cost value
   */
  virtual double ComputeCost(
      const F2CPoint& p1, const F2CPoint& p2, double ang2);

  /**
   * @brief Return the cost of going from swath s to point p.
   * @param[in] s Start point (end of the swath)
   * @param[in] p End point
   * @return Cost value
   */
  virtual double ComputeCost(const F2CSwath& s, const F2CPoint& p);

  /**
   * @brief Return the cost of going from swath s to point p.
   * @param[in] s1 Start point (end of the swath)
   * @param[in] s2 End point (start of the swath)
   * @return Cost value
   */
  virtual double ComputeCost(const F2CSwath& s1, const F2CSwath& s2);

  /**
   * @brief Return the cost of going from swath s to point p.
   * @param[in] s Start point (end of the swath)
   * @param[in] p End point
   * @param[in] ang Angle of the robot in p
   * @return Cost value
   */
  virtual double ComputeCost(const F2CSwath& s, const F2CPoint& p, double ang);

  /**
   * @brief Return the cost of going from point p to swath s.
   * @param[in] p Start point
   * @param[in] s End point (start of the swath)
   * @return Cost value
   */
  virtual double ComputeCost(const F2CPoint& p, const F2CSwath& s);

  /**
   * @brief Return the cost of going from point p to swath s.
   * @param[in] p Start point
   * @param[in] ang Angle of the robot in p
   * @param[in] s End point (start of the swath)
   * @return Cost value
   */
  virtual double ComputeCost(const F2CPoint& p, double ang, const F2CSwath& s);

  /**
   * @brief Return the cost of covering all the points of ps.
   * @param[in] ps vector of points
   * @return Cost value
   */
  virtual double ComputeCost(const std::vector<F2CPoint>& ps);

  /**
   * @brief Return the cost of covering all the points of ps.
   * @param[in] ps vector of points
   * @return Cost value
   */
  virtual double ComputeCost(const F2CMultiPoint& ps);

  /**
   * @brief Return the cost of going from swath s to the first point of ps.
   * @param[in] s Start point (end of the swath)
   * @param[in] ps vector of points
   * @return Cost value
   */
  virtual double ComputeCost(const F2CSwath& s, const F2CMultiPoint& ps);

  /**
   * @brief Return the cost of going from the last swath of s to the first point of ps.
   * @param[in] s Start point (end of the last swath)
   * @param[in] ps vector of points (first point)
   * @return Cost value
   */
  virtual double ComputeCost(const F2CSwaths& s, const F2CMultiPoint& ps);

  /**
   * @brief Return the cost of going from the last point of ps to the swath s.
   * @param[in] ps vector of points (end point)
   * @param[in] s Start point (start of the swath)
   * @return Cost value
   */
  virtual double ComputeCost(const F2CMultiPoint& ps, const F2CSwath& s);

  /**
   * @brief Return the cost of going from the last point of ps to the first swath of s.
   * @param[in] ps vector of points (end point)
   * @param[in] s Start point (start of the first swath)
   * @return Cost value
   */
  virtual double ComputeCost(const F2CMultiPoint& ps, const F2CSwaths& s);

  /**
   * @brief Return the cost of covering a swath.
   * @param[in] s Swath
   * @return Cost value
   */
  virtual double ComputeCost(const F2CSwath& s);

  /**
   * @brief Return the cost of covering a vector of swaths.
   *
   * Costs of each swath + Cost of going from one to another. The order
   * may affect the cost.
   * @param[in] s Swaths
   * @return Cost value
   */
  virtual double ComputeCost(const F2CSwaths& swaths);

  /**
   * @brief Return the cost of covering a Route.
   *
   * Costs of each swath + Cost of going from one to another.
   * @param[in] r Route
   * @return Cost value
   */
  virtual double ComputeCost(const F2CRoute& r);
};


}  // namespace autonomy::planning::planner::fields2cover::obj

