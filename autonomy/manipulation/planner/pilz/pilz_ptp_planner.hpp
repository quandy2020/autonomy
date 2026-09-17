/*
 * Copyright 2026 The Openbot Authors
 *
 * Pilz industrial motion: PTP / LIN / CIRC with joint-space blending.
 */

#pragma once

#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief Pilz PTP: point-to-point joint motion with cosine time law.
 *
 * Respects request.velocity_scale and max_velocity / max_acceleration
 * via synchronized ATRAP (Pilz VelocityProfileATrap lite).
 */
class PilzPtpPlanner : public PlannerBase {
 public:
  /**
   * @brief Store the planner id.
   * @param[in] planner_id Registry name.
   * @return true.
   */
  bool Init(const std::string& planner_id) override;

  /**
   * @brief Plan a PTP joint move start → goal.
   * @param[in] request Matching DOF start / goal states.
   * @return Timed joint trajectory.
   */
  MotionPlanResponse Plan(const MotionPlanRequest& request) override;

 private:
  std::string planner_id_;
};

/**
 * @brief Cartesian straight-line (Pilz LIN) via IK samples + cosine time law.
 */
class PilzLinPlanner : public PlannerBase {
 public:
  /**
   * @brief Store the planner id.
   * @param[in] planner_id Registry name.
   * @return true.
   */
  bool Init(const std::string& planner_id) override;

  /**
   * @brief Plan a LIN Cartesian path using request kinematics.
   * @param[in] request Must provide kinematics and Cartesian goal.
   * @return Joint trajectory or IK failure.
   */
  MotionPlanResponse Plan(const MotionPlanRequest& request) override;

 private:
  std::string planner_id_;
};

/**
 * @brief Circular arc (Pilz CIRC): start → interim → goal define the plane/arc.
 *
 * Uses request.cartesian_waypoints[0] as interim when size ≥ 1, else fails.
 */
class PilzCircPlanner : public PlannerBase {
 public:
  /**
   * @brief Store the planner id.
   * @param[in] planner_id Registry name.
   * @return true.
   */
  bool Init(const std::string& planner_id) override;

  /**
   * @brief Plan a CIRC arc through an interim Cartesian waypoint.
   * @param[in] request Requires kinematics and cartesian_waypoints[0].
   * @return Joint trajectory or planning failure.
   */
  MotionPlanResponse Plan(const MotionPlanRequest& request) override;

 private:
  std::string planner_id_;
};

/**
 * @brief Pilz Sequence: chain PTP/LIN/CIRC with per-item blend radii.
 *
 * Uses @c MotionPlanRequest::sequence. Empty sequence with multiple
 * @c cartesian_waypoints falls back to successive LIN segments.
 */
class PilzSequencePlanner : public PlannerBase {
 public:
  bool Init(const std::string& planner_id) override;
  MotionPlanResponse Plan(const MotionPlanRequest& request) override;

 private:
  std::string planner_id_;
};

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
