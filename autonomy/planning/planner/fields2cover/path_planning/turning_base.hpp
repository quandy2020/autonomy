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
 * @file turning_base.hpp
 * @brief Fields2Cover turning base.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <memory>
#include <map>
#include <vector>
#include <limits>
#include <functional>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/utils/random.hpp"

namespace autonomy::planning::planner::fields2cover::pp {

/**
 * @class autonomy::planning::planner::fields2cover::pp::TurningBase
 * @brief Base class for turn planners.
 */
class TurningBase {
 public:
  /**
   * @brief Create a turn that goes from one point with a certain angle to
   * another point.
   *
   * Start and end point are connected with a line that creates the
   * inferior border of the turn.
   * @param[in] robot Robot kinematic limits.
   * @param[in] start_pos Start point.
   * @param[in] start_angle Start angle.
   * @param[in] end_pos End point.
   * @param[in] end_angle End angle.
   * @return Path with the computed turn.
   */
  F2CPath CreateTurn(const F2CRobot& robot,
      const F2CPoint& start_pos, double start_angle,
      const F2CPoint& end_pos, double end_angle);

  /**
   * @brief Generate a turn if it has not been computed before.
   * @param[in] dist_start_pos Distance between start and end point
   * @param[in] start_angle Angle when going into the headland
   * (0 deg is the angle of the headland)
   * @param[in] end_angle Angle when going out of the headland
   */
  F2CPath CreateTurnIfNotCached(const F2CRobot& robot, double dist_start_pos,
      double start_angle, double end_angle);

  /**
   * @brief Create a turn.
   * @param[in] dist_start_pos Distance between start and end point
   * @param[in] start_angle Angle when going into the headland
   * (0 deg is the angle of the headland)
   * @param[in] end_angle Angle when going out of the headland
   */
  virtual F2CPath CreateSimpleTurn(const F2CRobot& robot, double dist_start_pos,
      double start_angle, double end_angle) = 0;

  /**
   * @brief Check if the turns keep the curvature continuous.
   */
  virtual bool HasContinuousCurvature() const;

  /**
   * @brief Transform the turn parameters representation from two points with
   * two angles to one Distance and two angles.
   * @param[in] start_pos Start point
   * @param[in] start_angle Start angle
   * @param[in] end_pos End point
   * @param[in] end_angle End angle
   * @return Vector with the values of the new representation
   */
  static std::vector<double> TransformToNormalTurn(const F2CPoint& start_pos,
      double start_angle, const F2CPoint& end_pos, double end_angle);

  /**
   * @brief Check if turn is valid.
   */
  static bool IsTurnValid(const F2CPath& path, double dist_start_end,
      double end_angle, double max_dist_error = 0.05,
      double max_rot_error = 0.1);


  /**
   * @brief Get discretization Distance from points in the turn.
   */
  double GetDiscretization() const;
  /**
   * @brief Set discretization Distance from points in the turn.
   */
  void SetDiscretization(double d);

  /**
   * @brief Get if turns are being cached or not.
   */
  bool GetUsingCache() const;
  /**
   * @brief Set if cache should be used when planning same turn as before.
   */
  void SetUsingCache(bool c);

  virtual ~TurningBase() = default;


 private:
  static void CorrectPath(F2CPath& path,
      const F2CPoint& start_pos,
      const F2CPoint& end_pos,
      float max_error_dist = 0.05);

 protected:
  // To prevent memory consumption and comparative errors because of doubles
  // ints are used multiplied by 1000.
  std::map<std::vector<int>, F2CPath> path_cache_;
  double discretization {0.01};
  bool using_cache {true};
};

}  // namespace autonomy::planning::planner::fields2cover::pp

