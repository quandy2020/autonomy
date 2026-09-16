/*
 * Copyright 2026 The Openbot Authors
 *
 * TaskApp that forwards ManipulationAction goals to the manipulation stack.
 */

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/task/common/typed_task.hpp"
#include <automsgs/task/manipulation.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace task {

class ManipulationTask
    : public TypedTaskAppBase<::autonomy::task::proto::ManipulationAction::Goal,
                              ::autonomy::task::proto::ManipulationAction::Feedback,
                              ::autonomy::task::proto::ManipulationAction::Result>
{
 public:
  AUTONOMY_SMART_PTR_DEFINITIONS(ManipulationTask)

  ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType() const override;

 protected:
  bool OnInitialize(
      const ::autonomy::task::proto::TaskServerOptions& options) override;
  bool OnGoal(
      const ::autonomy::task::proto::ManipulationAction::Goal& goal) override;
  void FillFeedback(
      ::autonomy::task::proto::ManipulationAction::Feedback* feedback)
      const override;
  void FillResult(
      ::autonomy::task::proto::ManipulationAction::Result* result) const override;

 private:
  std::optional<::autonomy::task::proto::ManipulationAction::Goal> active_goal_;
  std::string last_error_;
  bool last_ok_ = false;
};

}  // namespace task
}  // namespace autonomy
