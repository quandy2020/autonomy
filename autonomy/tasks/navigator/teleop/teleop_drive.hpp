/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/tasks/common/bt_navigator.hpp"
#include "autonomy/tasks/proto/bt_action.pb.h"
#include "autonomy/tasks/navigator/teleop/teleop_session.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace teleop {

/**
 * @brief BT-based teleop navigator (@ref AssistedTeleopAction + TeleopDrive BT node).
 */
class TeleopDriveNavigator
    : public common::BtNavigator<
          proto::AssistedTeleopAction>
{
public:
  using ActionT = proto::AssistedTeleopAction;

  AUTONOMY_SMART_PTR_DEFINITIONS(TeleopDriveNavigator)

  TeleopDriveNavigator(
    const proto::TaskOptions & options,
    const std::shared_ptr<common::TaskContext> & task_context,
    const std::vector<std::string> & plugin_lib_names,
    const common::FeedbackUtils & feedback_utils,
    const std::shared_ptr<common::NavigatorMuxer> & muxer,
    std::shared_ptr<common::OdomSmoother> odom_smoother);

  void setRunLimits(double max_linear_vel, double max_angular_vel);

  std::shared_ptr<TeleopSession> session() const;

  bool GoalReceived(std::shared_ptr<const typename ActionT::Goal> goal) override;
  void OnLoop() override;
  void OnPreempt(std::shared_ptr<const typename ActionT::Goal> goal) override;
  void GoalCompleted(std::shared_ptr<typename ActionT::Result> result,
                     const common::BtStatus final_bt_status) override;

  std::chrono::steady_clock::time_point start_time_;
  double run_max_linear_vel_{0.0};
  double run_max_angular_vel_{0.0};
  std::shared_ptr<TeleopSession> teleop_session_;
  std::shared_ptr<common::TaskContext> task_context_;
  proto::TaskOptions options_;
};

}  // namespace teleop
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
