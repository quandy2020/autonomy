/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/pipeline/motion_plan_message_conversion.hpp"

#include "autonomy/manipulation/model/joint_state_utilities.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {
namespace {

automsgs::msgs::moveit_msgs::MoveItErrorCodes::Code ToProtoCode(ErrorCode code) {
  using C = automsgs::msgs::moveit_msgs::MoveItErrorCodes;
  switch (code) {
    case ErrorCode::SUCCESS:
      return C::SUCCESS;
    case ErrorCode::PLANNING_FAILED:
      return C::PLANNING_FAILED;
    case ErrorCode::INVALID_MOTION_PLAN:
      return C::INVALID_MOTION_PLAN;
    case ErrorCode::CONTROL_FAILED:
      return C::CONTROL_FAILED;
    case ErrorCode::TIMED_OUT:
      return C::TIMED_OUT;
    case ErrorCode::PREEMPTED:
      return C::PREEMPTED;
    case ErrorCode::START_STATE_IN_COLLISION:
      return C::START_STATE_IN_COLLISION;
    case ErrorCode::GOAL_IN_COLLISION:
      return C::GOAL_IN_COLLISION;
    case ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION:
      return C::NO_IK_SOLUTION;
    case ErrorCode::INVALID_GROUP_NAME:
      return C::INVALID_GROUP_NAME;
    case ErrorCode::INVALID_ROBOT_STATE:
      return C::INVALID_ROBOT_STATE;
    case ErrorCode::FAILURE:
    default:
      return C::FAILURE;
  }
}

ErrorCode FromProtoCode(
    automsgs::msgs::moveit_msgs::MoveItErrorCodes::Code code) {
  using C = automsgs::msgs::moveit_msgs::MoveItErrorCodes;
  switch (code) {
    case C::SUCCESS:
      return ErrorCode::SUCCESS;
    case C::PLANNING_FAILED:
      return ErrorCode::PLANNING_FAILED;
    case C::INVALID_MOTION_PLAN:
      return ErrorCode::INVALID_MOTION_PLAN;
    case C::CONTROL_FAILED:
      return ErrorCode::CONTROL_FAILED;
    case C::TIMED_OUT:
      return ErrorCode::TIMED_OUT;
    case C::PREEMPTED:
      return ErrorCode::PREEMPTED;
    case C::START_STATE_IN_COLLISION:
      return ErrorCode::START_STATE_IN_COLLISION;
    case C::GOAL_IN_COLLISION:
      return ErrorCode::GOAL_IN_COLLISION;
    case C::NO_IK_SOLUTION:
      return ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION;
    case C::INVALID_GROUP_NAME:
      return ErrorCode::INVALID_GROUP_NAME;
    case C::INVALID_ROBOT_STATE:
      return ErrorCode::INVALID_ROBOT_STATE;
    default:
      return ErrorCode::FAILURE;
  }
}

void PoseToMessage(const automsgs::msgs::geometry_msgs::Pose& pose,
               automsgs::msgs::geometry_msgs::Pose* out) {
  if (!out) {
    return;
  }
  *out = pose;
}

void PoseFromMessage(const automsgs::msgs::geometry_msgs::Pose& msg,
                 automsgs::msgs::geometry_msgs::Pose* out) {
  if (!out) {
    return;
  }
  *out = msg;
}

}  // namespace

automsgs::msgs::moveit_msgs::MotionPlanRequest ToMessage(
    const MotionPlanRequest& request) {
  automsgs::msgs::moveit_msgs::MotionPlanRequest msg;
  msg.set_group_name(request.pb.group());
  msg.set_planner_id(request.pb.planner_id());
  *msg.mutable_start_state() = scene::ToMessage(request.pb.start_state());
  *msg.mutable_goal_state() = scene::ToMessage(request.pb.goal_state());
  if (request.pb.has_goal_pose()) {
    PoseToMessage(request.pb.goal_pose(), msg.mutable_goal_pose()->mutable_pose());
  }
  for (const auto& wp : request.pb.cartesian_waypoints()) {
    PoseToMessage(wp, msg.add_cartesian_waypoints()->mutable_pose());
  }
  msg.set_allowed_planning_time(request.pb.planning_time());
  msg.set_num_planning_attempts(request.pb.max_attempts());
  msg.set_max_velocity_scaling_factor(request.pb.velocity_scale());
  msg.set_max_acceleration_scaling_factor(request.pb.acceleration_scale());
  for (const auto& jc : request.pb.joint_constraints()) {
    *msg.mutable_path_constraints()->add_joint_constraints() = jc;
  }
  for (const auto& pc : request.pb.position_constraints()) {
    *msg.mutable_path_constraints()->add_position_constraints() = pc;
  }
  for (const auto& oc : request.pb.orientation_constraints()) {
    *msg.mutable_path_constraints()->add_orientation_constraints() = oc;
  }
  return msg;
}

MotionPlanRequest FromMessage(
    const automsgs::msgs::moveit_msgs::MotionPlanRequest& msg) {
  MotionPlanRequest request;
  request.pb.set_group(msg.group_name());
  request.pb.set_planner_id(msg.planner_id());
  *request.pb.mutable_start_state() = scene::FromMessage(msg.start_state());
  *request.pb.mutable_goal_state() = scene::FromMessage(msg.goal_state());
  request.pb.set_has_goal_pose(msg.has_goal_pose());
  if (msg.has_goal_pose()) {
    PoseFromMessage(msg.goal_pose().pose(), request.pb.mutable_goal_pose());
  }
  for (const auto& wp : msg.cartesian_waypoints()) {
    automsgs::msgs::geometry_msgs::Pose pose;
    PoseFromMessage(wp.pose(), &pose);
    *request.pb.add_cartesian_waypoints() = pose;
  }
  if (msg.allowed_planning_time() > 0) {
    request.pb.set_planning_time(msg.allowed_planning_time());
  }
  if (msg.num_planning_attempts() > 0) {
    request.pb.set_max_attempts(msg.num_planning_attempts());
  }
  if (msg.max_velocity_scaling_factor() > 0) {
    request.pb.set_velocity_scale(msg.max_velocity_scaling_factor());
  }
  if (msg.max_acceleration_scaling_factor() > 0) {
    request.pb.set_acceleration_scale(msg.max_acceleration_scaling_factor());
  }
  for (const auto& jc : msg.path_constraints().joint_constraints()) {
    *request.pb.add_joint_constraints() = jc;
  }
  for (const auto& pc : msg.path_constraints().position_constraints()) {
    *request.pb.add_position_constraints() = pc;
  }
  for (const auto& oc : msg.path_constraints().orientation_constraints()) {
    *request.pb.add_orientation_constraints() = oc;
  }
  return request;
}

automsgs::msgs::moveit_msgs::MotionPlanResponse ToMessage(
    const ::autonomy::manipulation::proto::MotionPlanResponse& response) {
  automsgs::msgs::moveit_msgs::MotionPlanResponse msg;
  msg.set_success(response.success());
  msg.mutable_error_code()->set_val(ToProtoCode(response.error_code()));
  msg.set_error(response.error());
  *msg.mutable_trajectory() = scene::ToMessage(response.trajectory());
  return msg;
}

::autonomy::manipulation::proto::MotionPlanResponse FromMessage(
    const automsgs::msgs::moveit_msgs::MotionPlanResponse& msg) {
  ::autonomy::manipulation::proto::MotionPlanResponse response;
  response.set_success(msg.success());
  response.set_error_code(FromProtoCode(msg.error_code().val()));
  response.set_error(msg.error());
  *response.mutable_trajectory() = scene::FromMessage(msg.trajectory());
  return response;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
