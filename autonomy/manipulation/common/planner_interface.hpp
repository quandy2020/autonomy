/*
 * Copyright 2026 The Openbot Authors
 *
 * Motion planner plugin base (MoveIt moveit_planners analogue).
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/model/link_fk.hpp"
#include "autonomy/manipulation/model/robot_model.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"
#include "autonomy/manipulation/common/msg_types.hpp"

namespace autonomy {
namespace manipulation {

namespace collision {
class CollisionDetector;
}  // namespace collision
namespace scene {
class PlanningScene;
}  // namespace scene

namespace planning {

/** @brief Alias: moveit_msgs/JointConstraint. */
using JointConstraint = ::autonomy::manipulation::JointConstraint;
/** @brief Alias: moveit_msgs/PositionConstraint. */
using PositionConstraint = ::autonomy::manipulation::PositionConstraint;
/** @brief Alias: moveit_msgs/OrientationConstraint. */
using OrientationConstraint = ::autonomy::manipulation::OrientationConstraint;

/**
 * @brief One Pilz Sequence item (PTP / LIN / CIRC) with blend radius.
 *
 * Mirrors moveit_msgs MotionSequenceItem (industrial subset).
 */
struct SequenceItem {
  /** @brief Command type: PTP / LIN / CIRC (or pilz_* aliases). */
  std::string type = "PTP";
  core::JointState goal_state;
  kinematics::Pose goal_pose;
  bool has_goal_pose = false;
  /** @brief CIRC interim pose (required for CIRC). */
  kinematics::Pose interim_pose;
  bool has_interim = false;
  double blend_radius = 0.0;
  double velocity_scale = 1.0;
};

/**
 * @brief Pilz Cartesian limits (MoveIt cartesian_limits_parameters lite).
 *
 * When @p configured is false, LIN/CIRC fall back to
 * MotionPlanRequest::max_velocity / max_acceleration.
 */
struct CartesianLimits {
  double max_trans_vel = 1.0;   // m/s
  double max_trans_acc = 2.0;   // m/s²
  double max_trans_dec = 2.0;   // m/s²
  double max_rot_vel = 1.0;     // rad/s
  bool configured = false;
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
  /** @brief Optional FK tree for full-chain CHOMP / collision. */
  std::shared_ptr<const core::LinkFkTree> link_tree;
  double max_velocity = 1.0;
  double max_acceleration = 2.0;
  /** @brief Pilz / industrial velocity scaling in (0, 1]. */
  double velocity_scale = 1.0;
  /** @brief Pilz acceleration scaling in (0, 1] (MoveIt acceleration_scaling_factor). */
  double acceleration_scale = 1.0;
  /** @brief Joint-space goal tolerance (rad / m) for OMPL GoalState. */
  double goal_joint_tolerance = 1e-3;
  /** @brief Cartesian tip goal tolerance (m) when CSS / pose goals used. */
  double goal_position_tolerance = 1e-3;
  /**
   * @brief Pilz blend radius (Cartesian m when FK available, else joint rad).
   * Zero disables blending at intermediate waypoints.
   */
  double blend_radius = 0.05;
  /** @brief Optional Cartesian velocity / acceleration limits for LIN/CIRC. */
  CartesianLimits cartesian_limits;
  /** @brief Pilz Sequence commands (planner_id = pilz_sequence). */
  std::vector<SequenceItem> sequence;
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
