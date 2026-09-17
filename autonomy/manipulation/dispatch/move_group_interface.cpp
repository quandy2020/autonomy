/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/move_group_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace interface {

MoveGroupInterface::MoveGroupInterface(ManipulationServer* server)
    : server_(server) {}

void MoveGroupInterface::SetPlannerId(const std::string& planner_id) {
  planner_id_ = planner_id;
}

void MoveGroupInterface::SetGroup(const std::string& group) {
  group_ = group;
}

planning::MotionPlanResponse MoveGroupInterface::PlanToJointTarget(
    const core::JointState& goal) {
  planning::MotionPlanRequest request;
  request.planner_id = planner_id_;
  request.group = group_;
  request.goal_state = goal;
  if (!server_) {
    planning::MotionPlanResponse response;
    response.error = "no server";
    response.error_code = ErrorCode::kFailure;
    return response;
  }
  return server_->Plan(request);
}

planning::MotionPlanResponse MoveGroupInterface::PlanToPoseTarget(
    const kinematics::Pose& goal) {
  planning::MotionPlanRequest request;
  request.planner_id = planner_id_;
  request.group = group_;
  request.goal_pose = goal;
  request.has_goal_pose = true;
  if (!server_) {
    planning::MotionPlanResponse response;
    response.error = "no server";
    response.error_code = ErrorCode::kFailure;
    return response;
  }
  return server_->Plan(request);
}

planning::MotionPlanResponse MoveGroupInterface::PlanAndExecute(
    const planning::MotionPlanRequest& request) {
  if (!server_) {
    planning::MotionPlanResponse response;
    response.error = "no server";
    response.error_code = ErrorCode::kFailure;
    return response;
  }
  auto* cap = dynamic_cast<server::PlanAndExecuteCapability*>(
      server_->GetCapability("plan_and_execute"));
  if (!cap) {
    planning::MotionPlanResponse response;
    response.error = "no plan_and_execute capability";
    response.error_code = ErrorCode::kFailure;
    return response;
  }
  return cap->Run(request);
}

}  // namespace interface
}  // namespace manipulation
}  // namespace autonomy
