/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/manipulation/manipulation.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/task/common/names.hpp"

namespace autonomy {
namespace task {

::automsgs::msgs::vehicle_msgs::RobotTaskType ManipulationTask::GetTaskType()
    const {
  return ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_MANIPULATION;
}

void ManipulationTask::SetNode(std::shared_ptr<autolink::Node> /*node*/) {}

bool ManipulationTask::OnInitialize(
    const ::autonomy::task::proto::TaskServerOptions& /*options*/) {
  AINFO << "ManipulationTask ready; action bridge target="
        << kManipulationMove;
  return true;
}

bool ManipulationTask::OnGoal(
    const ::autonomy::task::proto::ManipulationAction::Goal& goal) {
  active_goal_ = goal;
  last_ok_ = true;
  last_error_.clear();
  // Goals are executed by autonomy.manipulation Action server; this TaskApp
  // records intent for scheduler / bridge visibility.
  AINFO << "ManipulationTask accepted id=" << goal.id()
        << " group=" << goal.group() << " planner=" << goal.planner()
        << " has_motion_plan=" << goal.has_motion_plan()
        << " replace_execution=" << goal.replace_execution();
  return true;
}

void ManipulationTask::FillFeedback(
    ::autonomy::task::proto::ManipulationAction::Feedback* feedback) const {
  if (!feedback) {
    return;
  }
  feedback->set_status(::autonomy::task::proto::RUNNING);
  feedback->set_progress(active_goal_ ? 0.5f : 0.0f);
}

void ManipulationTask::FillResult(
    ::autonomy::task::proto::ManipulationAction::Result* result) const {
  if (!result) {
    return;
  }
  result->set_status(last_ok_ ? ::autonomy::task::proto::SUCCEEDED
                              : ::autonomy::task::proto::FAILED);
  result->set_error(last_error_);
  result->set_error_code(last_ok_ ? 1 : 99999);
}

}  // namespace task
}  // namespace autonomy
