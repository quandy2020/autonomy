/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planning/plan_convert.hpp"

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

}  // namespace

automsgs::msgs::moveit_msgs::MotionPlanRequest ToMsg(
    const MotionPlanRequest& request) {
  automsgs::msgs::moveit_msgs::MotionPlanRequest msg;
  msg.set_group_name(request.group);
  msg.set_planner_id(request.planner_id);
  *msg.mutable_start_state() = scene::ToMsg(request.start_state);
  *msg.mutable_goal_state() = scene::ToMsg(request.goal_state);
  if (request.has_goal_pose) {
    auto* pose = msg.mutable_goal_pose()->mutable_pose();
    pose->mutable_position()->set_x(request.goal_pose.x);
    pose->mutable_position()->set_y(request.goal_pose.y);
    pose->mutable_position()->set_z(request.goal_pose.z);
    pose->mutable_orientation()->set_x(request.goal_pose.qx);
    pose->mutable_orientation()->set_y(request.goal_pose.qy);
    pose->mutable_orientation()->set_z(request.goal_pose.qz);
    pose->mutable_orientation()->set_w(request.goal_pose.qw);
  }
  for (const auto& wp : request.cartesian_waypoints) {
    auto* p = msg.add_cartesian_waypoints()->mutable_pose();
    p->mutable_position()->set_x(wp.x);
    p->mutable_position()->set_y(wp.y);
    p->mutable_position()->set_z(wp.z);
    p->mutable_orientation()->set_x(wp.qx);
    p->mutable_orientation()->set_y(wp.qy);
    p->mutable_orientation()->set_z(wp.qz);
    p->mutable_orientation()->set_w(wp.qw);
  }
  msg.set_allowed_planning_time(request.planning_time);
  msg.set_num_planning_attempts(request.max_attempts);
  msg.set_max_velocity_scaling_factor(request.velocity_scale);
  for (const auto& jc : request.joint_constraints) {
    auto* c = msg.mutable_path_constraints()->add_joint_constraints();
    c->set_joint_name(jc.joint_name);
    c->set_position(jc.position);
    c->set_tolerance_above(jc.tolerance_above);
    c->set_tolerance_below(jc.tolerance_below);
  }
  for (const auto& pc : request.position_constraints) {
    auto* c = msg.mutable_path_constraints()->add_position_constraints();
    c->set_link_name(pc.link_name);
    c->set_tolerance(pc.tolerance);
    auto* pose = c->mutable_target()->mutable_pose();
    pose->mutable_position()->set_x(pc.target.x);
    pose->mutable_position()->set_y(pc.target.y);
    pose->mutable_position()->set_z(pc.target.z);
    pose->mutable_orientation()->set_x(pc.target.qx);
    pose->mutable_orientation()->set_y(pc.target.qy);
    pose->mutable_orientation()->set_z(pc.target.qz);
    pose->mutable_orientation()->set_w(pc.target.qw);
  }
  for (const auto& oc : request.orientation_constraints) {
    auto* c = msg.mutable_path_constraints()->add_orientation_constraints();
    c->set_link_name(oc.link_name);
    c->set_tolerance(oc.tolerance);
    auto* pose = c->mutable_target()->mutable_pose();
    pose->mutable_position()->set_x(oc.target.x);
    pose->mutable_position()->set_y(oc.target.y);
    pose->mutable_position()->set_z(oc.target.z);
    pose->mutable_orientation()->set_x(oc.target.qx);
    pose->mutable_orientation()->set_y(oc.target.qy);
    pose->mutable_orientation()->set_z(oc.target.qz);
    pose->mutable_orientation()->set_w(oc.target.qw);
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
    const auto& p = msg.goal_pose().pose();
    request.goal_pose.x = p.position().x();
    request.goal_pose.y = p.position().y();
    request.goal_pose.z = p.position().z();
    request.goal_pose.qx = p.orientation().x();
    request.goal_pose.qy = p.orientation().y();
    request.goal_pose.qz = p.orientation().z();
    request.goal_pose.qw = p.orientation().w();
  }
  for (const auto& wp : msg.cartesian_waypoints()) {
    kinematics::Pose pose;
    pose.x = wp.pose().position().x();
    pose.y = wp.pose().position().y();
    pose.z = wp.pose().position().z();
    pose.qx = wp.pose().orientation().x();
    pose.qy = wp.pose().orientation().y();
    pose.qz = wp.pose().orientation().z();
    pose.qw = wp.pose().orientation().w();
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
  for (const auto& jc : msg.path_constraints().joint_constraints()) {
    JointConstraint c;
    c.joint_name = jc.joint_name();
    c.position = jc.position();
    c.tolerance_above = jc.tolerance_above() > 0 ? jc.tolerance_above() : 0.01;
    c.tolerance_below = jc.tolerance_below() > 0 ? jc.tolerance_below() : 0.01;
    request.joint_constraints.push_back(c);
  }
  for (const auto& pc : msg.path_constraints().position_constraints()) {
    PositionConstraint c;
    c.link_name = pc.link_name();
    c.tolerance = pc.tolerance() > 0 ? pc.tolerance() : 0.01;
    const auto& p = pc.target().pose();
    c.target.x = p.position().x();
    c.target.y = p.position().y();
    c.target.z = p.position().z();
    c.target.qx = p.orientation().x();
    c.target.qy = p.orientation().y();
    c.target.qz = p.orientation().z();
    c.target.qw = p.orientation().w();
    request.position_constraints.push_back(c);
  }
  for (const auto& oc : msg.path_constraints().orientation_constraints()) {
    OrientationConstraint c;
    c.link_name = oc.link_name();
    c.tolerance = oc.tolerance() > 0 ? oc.tolerance() : 0.05;
    const auto& p = oc.target().pose();
    c.target.x = p.position().x();
    c.target.y = p.position().y();
    c.target.z = p.position().z();
    c.target.qx = p.orientation().x();
    c.target.qy = p.orientation().y();
    c.target.qz = p.orientation().z();
    c.target.qw = p.orientation().w();
    request.orientation_constraints.push_back(c);
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
