/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/pipeline/plan_convert.hpp"

#include "autonomy/manipulation/common/joint_state_util.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

automsgs::msgs::moveit_msgs::MoveItErrorCodes::Code ToProtoCode(ErrorCode code) {
  using C = automsgs::msgs::moveit_msgs::MoveItErrorCodes;
  switch (code) {
    case ErrorCode::kSuccess:
      return C::SUCCESS;
    case ErrorCode::kPlanningFailed:
      return C::PLANNING_FAILED;
    case ErrorCode::kInvalidMotionPlan:
      return C::INVALID_MOTION_PLAN;
    case ErrorCode::kControlFailed:
      return C::CONTROL_FAILED;
    case ErrorCode::kTimedOut:
      return C::TIMED_OUT;
    case ErrorCode::kPreempted:
      return C::PREEMPTED;
    case ErrorCode::kStartStateInCollision:
      return C::START_STATE_IN_COLLISION;
    case ErrorCode::kGoalInCollision:
      return C::GOAL_IN_COLLISION;
    case ErrorCode::kNoIkSolution:
      return C::NO_IK_SOLUTION;
    case ErrorCode::kInvalidGroupName:
      return C::INVALID_GROUP_NAME;
    case ErrorCode::kInvalidRobotState:
      return C::INVALID_ROBOT_STATE;
    case ErrorCode::kFailure:
    default:
      return C::FAILURE;
  }
}

ErrorCode FromProtoCode(
    automsgs::msgs::moveit_msgs::MoveItErrorCodes::Code code) {
  using C = automsgs::msgs::moveit_msgs::MoveItErrorCodes;
  switch (code) {
    case C::SUCCESS:
      return ErrorCode::kSuccess;
    case C::PLANNING_FAILED:
      return ErrorCode::kPlanningFailed;
    case C::INVALID_MOTION_PLAN:
      return ErrorCode::kInvalidMotionPlan;
    case C::CONTROL_FAILED:
      return ErrorCode::kControlFailed;
    case C::TIMED_OUT:
      return ErrorCode::kTimedOut;
    case C::PREEMPTED:
      return ErrorCode::kPreempted;
    case C::START_STATE_IN_COLLISION:
      return ErrorCode::kStartStateInCollision;
    case C::GOAL_IN_COLLISION:
      return ErrorCode::kGoalInCollision;
    case C::NO_IK_SOLUTION:
      return ErrorCode::kNoIkSolution;
    case C::INVALID_GROUP_NAME:
      return ErrorCode::kInvalidGroupName;
    case C::INVALID_ROBOT_STATE:
      return ErrorCode::kInvalidRobotState;
    default:
      return ErrorCode::kFailure;
  }
}

void PoseToMsg(const kinematics::Pose& pose,
               automsgs::msgs::geometry_msgs::Pose* out) {
  if (!out) {
    return;
  }
  *out = pose;
}

void PoseFromMsg(const automsgs::msgs::geometry_msgs::Pose& msg,
                 kinematics::Pose* out) {
  if (!out) {
    return;
  }
  *out = msg;
}

}  // namespace

automsgs::msgs::moveit_msgs::MotionPlanRequest ToMsg(
    const MotionPlanRequest& request) {
  automsgs::msgs::moveit_msgs::MotionPlanRequest msg;
  msg.set_group_name(request.group);
  msg.set_planner_id(request.planner_id);
  *msg.mutable_start_state() = scene::ToMsg(request.start_state);
  *msg.mutable_goal_state() = scene::ToMsg(request.goal_state);
  if (request.has_goal_pose) {
    PoseToMsg(request.goal_pose, msg.mutable_goal_pose()->mutable_pose());
  }
  for (const auto& wp : request.cartesian_waypoints) {
    PoseToMsg(wp, msg.add_cartesian_waypoints()->mutable_pose());
  }
  msg.set_allowed_planning_time(request.planning_time);
  msg.set_num_planning_attempts(request.max_attempts);
  msg.set_max_velocity_scaling_factor(request.velocity_scale);
  msg.set_max_acceleration_scaling_factor(request.acceleration_scale);
  for (const auto& jc : request.joint_constraints) {
    *msg.mutable_path_constraints()->add_joint_constraints() = jc;
  }
  for (const auto& pc : request.position_constraints) {
    *msg.mutable_path_constraints()->add_position_constraints() = pc;
  }
  for (const auto& oc : request.orientation_constraints) {
    *msg.mutable_path_constraints()->add_orientation_constraints() = oc;
  }
  return msg;
}

MotionPlanRequest FromMsg(
    const automsgs::msgs::moveit_msgs::MotionPlanRequest& msg) {
  MotionPlanRequest request;
  request.group = msg.group_name();
  request.planner_id = msg.planner_id();
  request.start_state = scene::FromMsg(msg.start_state());
  request.goal_state = scene::FromMsg(msg.goal_state());
  request.has_goal_pose = msg.has_goal_pose();
  if (msg.has_goal_pose()) {
    PoseFromMsg(msg.goal_pose().pose(), &request.goal_pose);
  }
  for (const auto& wp : msg.cartesian_waypoints()) {
    kinematics::Pose pose;
    PoseFromMsg(wp.pose(), &pose);
    request.cartesian_waypoints.push_back(pose);
  }
  if (msg.allowed_planning_time() > 0) {
    request.planning_time = msg.allowed_planning_time();
  }
  if (msg.num_planning_attempts() > 0) {
    request.max_attempts = msg.num_planning_attempts();
  }
  if (msg.max_velocity_scaling_factor() > 0) {
    request.velocity_scale = msg.max_velocity_scaling_factor();
  }
  if (msg.max_acceleration_scaling_factor() > 0) {
    request.acceleration_scale = msg.max_acceleration_scaling_factor();
  }
  for (const auto& jc : msg.path_constraints().joint_constraints()) {
    request.joint_constraints.push_back(jc);
  }
  for (const auto& pc : msg.path_constraints().position_constraints()) {
    request.position_constraints.push_back(pc);
  }
  for (const auto& oc : msg.path_constraints().orientation_constraints()) {
    request.orientation_constraints.push_back(oc);
  }
  return request;
}

automsgs::msgs::moveit_msgs::MotionPlanResponse ToMsg(
    const MotionPlanResponse& response) {
  automsgs::msgs::moveit_msgs::MotionPlanResponse msg;
  msg.set_success(response.success);
  msg.mutable_error_code()->set_val(ToProtoCode(response.error_code));
  msg.set_error(response.error);
  *msg.mutable_trajectory() = scene::ToMsg(response.trajectory);
  return msg;
}

MotionPlanResponse FromMsg(
    const automsgs::msgs::moveit_msgs::MotionPlanResponse& msg) {
  MotionPlanResponse response;
  response.success = msg.success();
  response.error_code = FromProtoCode(msg.error_code().val());
  response.error = msg.error();
  response.trajectory = scene::FromMsg(msg.trajectory());
  return response;
}

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
