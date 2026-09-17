/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/move_group_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

MoveGroupInterface::MoveGroupInterface(ManipulationServer* server)
    : server_(server) {}

void MoveGroupInterface::SetPlannerId(const std::string& planner_id) {
  planner_id_ = planner_id;
}

void MoveGroupInterface::SetGroup(const std::string& group) {
  group_ = group;
}

::autonomy::manipulation::proto::MotionPlanResponse MoveGroupInterface::PlanToJointTarget(
    const automsgs::msgs::sensor_msgs::JointState& goal) {
  planner::MotionPlanRequest request;
  request.pb.set_planner_id(planner_id_);
  request.pb.set_group(group_);
  *request.pb.mutable_goal_state() = goal;
  if (!server_) {
    ::autonomy::manipulation::proto::MotionPlanResponse response;
    response.set_error("no server");
    response.set_error_code(ErrorCode::FAILURE);
    return response;
  }
  return server_->Plan(request);
}

::autonomy::manipulation::proto::MotionPlanResponse MoveGroupInterface::PlanToPoseTarget(
    const automsgs::msgs::geometry_msgs::Pose& goal) {
  planner::MotionPlanRequest request;
  request.pb.set_planner_id(planner_id_);
  request.pb.set_group(group_);
  *request.pb.mutable_goal_pose() = goal;
  request.pb.set_has_goal_pose(true);
  if (!server_) {
    ::autonomy::manipulation::proto::MotionPlanResponse response;
    response.set_error("no server");
    response.set_error_code(ErrorCode::FAILURE);
    return response;
  }
  return server_->Plan(request);
}

::autonomy::manipulation::proto::MotionPlanResponse MoveGroupInterface::PlanAndExecute(
    const planner::MotionPlanRequest& request) {
  if (!server_) {
    ::autonomy::manipulation::proto::MotionPlanResponse response;
    response.set_error("no server");
    response.set_error_code(ErrorCode::FAILURE);
    return response;
  }
  auto* cap = dynamic_cast<dispatch::PlanAndExecuteCapability*>(
      server_->GetCapability("plan_and_execute"));
  if (!cap) {
    ::autonomy::manipulation::proto::MotionPlanResponse response;
    response.set_error("no plan_and_execute capability");
    response.set_error_code(ErrorCode::FAILURE);
    return response;
  }
  return cap->Run(request);
}

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
