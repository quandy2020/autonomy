/*
 * Copyright 2026 The Openbot Authors
 *
 * Motion planner plugin base (MoveIt moveit_planners analogue).
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/manipulation/core/error_codes.hpp"
#include "autonomy/manipulation/core/robot_model.hpp"
#include "autonomy/manipulation/kinematics/kinematics_base.hpp"

namespace autonomy {
namespace manipulation {

namespace collision {
class CollisionDetector;
}  // namespace collision
namespace scene {
class PlanningScene;
}  // namespace scene

namespace planning {

/** @brief Single-joint goal or path constraint. */
struct JointConstraint {
  std::string joint_name;
  double position = 0.0;
  double tolerance_above = 0.01;
  double tolerance_below = 0.01;
};

/** @brief Cartesian position bound on a link tip. */
struct PositionConstraint {
  std::string link_name;
  kinematics::Pose target;
  double tolerance = 0.01;
};

/** @brief Orientation bound on a link tip (tolerance in radians). */
struct OrientationConstraint {
  std::string link_name;
  kinematics::Pose target;
  double tolerance = 0.05;
};

/** @brief Motion planning request shared by all planners. */
struct MotionPlanRequest {
  std::string group;
  std::string planner_id;
  core::JointState start_state;
  core::JointState goal_state;
  kinematics::Pose goal_pose;
  bool has_goal_pose = false;
  std::vector<kinematics::Pose> cartesian_waypoints;
  double planning_time = 5.0;  // s
  int max_attempts = 3;
  std::vector<JointConstraint> joint_constraints;
  std::vector<PositionConstraint> position_constraints;
  std::vector<OrientationConstraint> orientation_constraints;
  std::shared_ptr<scene::PlanningScene> scene;
  std::shared_ptr<collision::CollisionDetector> collision;
  std::shared_ptr<kinematics::KinematicsBase> kinematics;
  std::shared_ptr<core::RobotModel> model;
  double max_velocity = 1.0;
  double max_acceleration = 2.0;
  /** @brief Pilz / industrial velocity scaling in (0, 1]. */
  double velocity_scale = 1.0;
};

/** @brief Motion planning result: success flag, error, and trajectory. */
struct MotionPlanResponse {
  bool success = false;
  ErrorCode error_code = ErrorCode::kFailure;
  std::string error;
  core::RobotTrajectory trajectory;
};

/**
 * @brief Abstract motion planner plugin interface.
 *
 * Implementations are registered via the manipulation plugin hub and selected
 * by @c MotionPlanRequest::planner_id.
 */
class PlannerBase {
 public:
  virtual ~PlannerBase() = default;

  /**
   * @brief Initialize the planner with its registry id.
   * @param[in] planner_id Plugin / planner identifier.
   * @return true on success.
   */
  virtual bool Init(const std::string& planner_id) = 0;

  /**
   * @brief Compute a collision-aware plan for @p request.
   * @param[in] request Planning inputs (start, goal, scene, limits).
   * @return Trajectory and error code.
   */
  virtual MotionPlanResponse Plan(const MotionPlanRequest& request) = 0;
};

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
