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
 * @file robot.hpp
 * @brief Fields2Cover Robot.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <ogr_geometry.h>
#include <utility>
#include <string>
#include <vector>
#include <optional>
#include "autonomy/planning/planner/fields2cover/types/point.hpp"
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/multi_line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/cell.hpp"
#include "autonomy/planning/planner/fields2cover/types/cells.hpp"
#include "autonomy/planning/planner/fields2cover/types/path.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct Robot {
 public:
  explicit Robot(double width, double cov_width = 0.0,
      double max_curv = 1.0, double max_diff_curv = 0.3);
  Robot();
  ~Robot();
  Robot(const Robot&);
  Robot(Robot &&);
  Robot &operator=(const Robot&);
  Robot &operator=(Robot&&);

  std::string GetName() const;
  void SetName(const std::string& str);

  double GetWidth() const;
  void SetWidth(double);

  double GetCovWidth() const;
  void SetCovWidth(double);

  double GetCruiseVel() const;
  void SetCruiseVel(double);

  double GetTurnVel() const;
  void SetTurnVel(double);

  double GetMinTurningRadius() const;
  void SetMinTurningRadius(double rad);

  double GetMaxCurv() const;
  void SetMaxCurv(double);

  double GetMaxDiffCurv() const;
  void SetMaxDiffCurv(double);

  /**
   * @brief Radius swept by a turn with continuous curvature, wider than the minimum.
   */
  double GetSmoothTurningRadius() const;
  /**
   * @brief Radius a turn deflecting `sweep` radians is driven at.
   */
  double GetTurnRadius(double sweep, bool continuous) const;

  double GetMaxCornerCut() const;
  void SetMaxCornerCut(double);

  double GetDirectHopMaxDev() const;
  void SetDirectHopMaxDev(double);

  double GetUturnMaxHop() const;
  void SetUturnMaxHop(double);

  double GetTrackSimplifyTol() const;
  void SetTrackSimplifyTol(double);

  double GetReversalSweep() const;
  void SetReversalSweep(double);

  double GetTurnSlack() const;
  void SetTurnSlack(double);

  double GetMinSweep() const;
  void SetMinSweep(double);

  double GetLegShare() const;
  void SetLegShare(double);

  double GetRadiusMargin() const;
  void SetRadiusMargin(double);

  double GetMinBackoffRadii() const;
  void SetMinBackoffRadii(double);

  double GetUturnReachRadii() const;
  void SetUturnReachRadii(double);

  size_t GetMaxCornerSpan() const;
  void SetMaxCornerSpan(size_t);

 private:
  std::string name_;

  /**
   * @brief Width of the robot.
   */
  double width_ {0.0};
  /**
   * @brief Width of the coverage Area of the robot.
   */
  double cov_width_ {0.0};

  /**
   * @brief Velocity of the robot when not doing turns.
   */
  double cruise_speed_ {1.0};

  /**
   * @brief Maximum instantaneous curvature change.
   */
  double max_icc_ {1.0};  // [1/m]

  /**
   * @brief Maximum linear curvature change.
   */
  double linear_curv_change_ {0.3};  // [1/m^2]

  /**
   * @brief Velocity of the robot when doing turns. If not set, cruise_speed_ is used.
   */
  std::optional<double> turn_vel_;

  /**
   * @brief How far rounding a corner may leave its track. If not set, the wider of
   * the turning radius and half the operation width, capped at the width.
   */
  std::optional<double> max_corner_cut_;
  /**
   * @brief Below this deviation from the straight hop, a connection is driven
   * direct. If not set, a quarter of the operation width.
   */
  std::optional<double> direct_hop_max_dev_;
  /**
   * @brief Below this hop, a reversal is a u-turn rather than a detour to follow.
   * If not set, three operation widths.
   */
  std::optional<double> uturn_max_hop_;
  /**
   * @brief Points this close to the chord they would round are dropped before a
   * connection's corners are planned. If not set, a tenth of the width.
   */
  std::optional<double> track_simplify_tol_;

  /**
   * @brief Heading difference between two legs that reads as doubling back.
   */
  double reversal_sweep_ {2.0};  // [rad]
  /**
   * @brief Turning this much past a corner's deflection is a loop or an S, refused.
   */
  double turn_slack_ {1.0};  // [rad]
  /**
   * @brief Corners deflecting less than this are driven straight through.
   */
  double min_sweep_ {0.05};  // [rad]
  /**
   * @brief Share of a leg a corner may use, the rest left to the corner beyond it.
   */
  double leg_share_ {0.5};
  /**
   * @brief Room for a continuous turn's clothoid lead-in, in tangent lengths.
   */
  double radius_margin_ {1.2};
  /**
   * @brief Approach a shallow corner keeps even where its tangent collapses, in turn radii.
   */
  double min_backoff_radii_ {0.5};
  /**
   * @brief How far back a u-turn may start, in turn radii.
   */
  double uturn_reach_radii_ {4.0};
  /**
   * @brief Most corners folded into a single maneuver.
   */
  size_t max_corner_span_ {3};
};

}  // namespace autonomy::planning::planner::fields2cover::types

