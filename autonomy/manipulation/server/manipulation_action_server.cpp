/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/server/manipulation_action_server.hpp"

#include <cstdint>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/constants.hpp"
#include "autonomy/manipulation/core/error_codes.hpp"
#include "autonomy/manipulation/planning/plan_convert.hpp"
#include "autonomy/manipulation/scene/msg_convert.hpp"

namespace autonomy {
namespace manipulation {
namespace {

planning::MotionPlanRequest ToRequest(
    const ManipulationActionServer::ActionT::Goal& goal) {
  if (goal.has_motion_plan()) {
    auto request = planning::FromMsg(goal.motion_plan());
    if (request.group.empty() && !goal.group().empty()) {
      request.group = goal.group();
    }
    if (request.planner_id.empty() && !goal.planner().empty()) {
      request.planner_id = goal.planner();
    }
    return request;
  }

  planning::MotionPlanRequest request;
  request.group = goal.group();
  request.planner_id = goal.planner();
  request.goal_state.names.assign(goal.joint_names().begin(),
                                  goal.joint_names().end());
  request.goal_state.positions.assign(goal.joint_positions().begin(),
                                      goal.joint_positions().end());
  if (goal.has_pose()) {
    request.has_goal_pose = true;
    const auto& p = goal.pose().pose().position();
    const auto& q = goal.pose().pose().orientation();
    request.goal_pose.x = p.x();
    request.goal_pose.y = p.y();
    request.goal_pose.z = p.z();
    request.goal_pose.qx = q.x();
    request.goal_pose.qy = q.y();
    request.goal_pose.qz = q.z();
    request.goal_pose.qw = q.w();
  }
  return request;
}

void FillTrajectory(
    const core::RobotTrajectory& traj,
    ManipulationActionServer::ActionT::Result* result) {
  if (!result || traj.waypoints.empty()) {
    return;
  }
  *result->mutable_trajectory() = scene::ToMsg(traj);
}

}  // namespace

ManipulationActionServer::ManipulationActionServer(ManipulationServer* server)
    : server_(server) {}

bool ManipulationActionServer::Init(
    const std::shared_ptr<autolink::Node>& node,
    const std::string& action_name) {
  if (!server_ || !node) {
    return false;
  }
  const std::string name =
      action_name.empty() ? kManipulationAction : action_name;

  action_server_ = std::make_shared<ServerT>(
      node, name, [this]() { ExecuteCallback(); });
  AINFO << "ManipulationActionServer listening on " << name;
  return true;
}

void ManipulationActionServer::Shutdown() {
  action_server_.reset();
}

void ManipulationActionServer::ExecuteCallback() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!action_server_ || !server_) {
    return;
  }

  const auto goal = action_server_->GetCurrentGoal();
  if (!goal) {
    return;
  }

  auto feedback = std::make_shared<ActionT::Feedback>();
  feedback->set_status(::autonomy::task::proto::RUNNING);
  feedback->set_progress(0.0f);
  action_server_->PublishFeedback(feedback);

  if (goal->cmd() == ::autonomy::task::proto::CANCEL) {
    server_->CancelExecution();
    auto result = std::make_shared<ActionT::Result>();
    result->set_status(::autonomy::task::proto::CANCELED);
    result->set_error_code(static_cast<int32_t>(ErrorCode::kPreempted));
    action_server_->SucceededCurrent(result);
    return;
  }

  const auto request = ToRequest(*goal);
  const auto plan = server_->Plan(request);
  if (!plan.success) {
    auto result = std::make_shared<ActionT::Result>();
    result->set_status(::autonomy::task::proto::FAILED);
    result->set_error(plan.error);
    result->set_error_code(static_cast<int32_t>(plan.error_code));
    action_server_->TerminateCurrent(result);
    return;
  }

  feedback->set_progress(0.5f);
  action_server_->PublishFeedback(feedback);

  if (goal->execute()) {
    const ErrorCode code =
        server_->ExecuteTrajectory(plan.trajectory, goal->replace_execution());
    if (code != ErrorCode::kSuccess) {
      auto result = std::make_shared<ActionT::Result>();
      result->set_status(::autonomy::task::proto::FAILED);
      result->set_error(ErrorCodeName(code));
      result->set_error_code(static_cast<int32_t>(code));
      FillTrajectory(plan.trajectory, result.get());
      action_server_->TerminateCurrent(result);
      return;
    }
  }

  feedback->set_status(::autonomy::task::proto::SUCCEEDED);
  feedback->set_progress(1.0f);
  action_server_->PublishFeedback(feedback);

  auto result = std::make_shared<ActionT::Result>();
  result->set_status(::autonomy::task::proto::SUCCEEDED);
  result->set_error_code(static_cast<int32_t>(ErrorCode::kSuccess));
  FillTrajectory(plan.trajectory, result.get());
  action_server_->SucceededCurrent(result);
}

}  // namespace manipulation
}  // namespace autonomy
