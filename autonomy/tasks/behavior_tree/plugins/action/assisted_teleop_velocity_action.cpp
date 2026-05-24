/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/tasks/behavior_tree/plugins/action/assisted_teleop_velocity_action.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_session.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_velocity_filter.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

AssistedTeleopVelocityAction::AssistedTeleopVelocityAction(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BtStatefulActionNode(xml_tag_name, conf)
{
}

BT::NodeStatus AssistedTeleopVelocityAction::onStart()
{
  auto ctx = taskContext();
  if (!ctx || !ctx->teleop_session) {
    AERROR << "AssistedTeleopVelocity: task_context or teleop_session missing";
    return BT::NodeStatus::FAILURE;
  }
  session_ = ctx->teleop_session;

  time_allowance_sec_ = 0.0;
  getInput("time_allowance", time_allowance_sec_);
  disable_collision_checks_ = false;
  getInput("disable_collision_checks", disable_collision_checks_);
  projection_time_sec_ = 1.5;
  simulation_step_sec_ = 0.1;
  getInput("projection_time_sec", projection_time_sec_);
  getInput("simulation_step_sec", simulation_step_sec_);

  double max_linear = 0.0;
  double max_angular = 0.0;
  getInput("max_linear_vel", max_linear);
  getInput("max_angular_vel", max_angular);

  if (config().blackboard) {
    config().blackboard->get("teleop_max_linear_vel", max_linear);   // NOLINT
    config().blackboard->get("teleop_max_angular_vel", max_angular);  // NOLINT
    if (time_allowance_sec_ <= 0.0) {
      config().blackboard->get("teleop_time_allowance", time_allowance_sec_);  // NOLINT
    }
    config().blackboard->get("teleop_disable_collision_checks",  // NOLINT
                             disable_collision_checks_);
    config().blackboard->get("teleop_projection_time_sec",  // NOLINT
                             projection_time_sec_);
    config().blackboard->get("teleop_simulation_step_sec",  // NOLINT
                             simulation_step_sec_);
  }

  navigator::teleop::TeleopSession::Limits limits;
  if (config().blackboard) {
    config().blackboard->get("teleop_cmd_stale_timeout_sec",  // NOLINT
                            limits.cmd_stale_timeout_sec);
  }
  if (max_linear > 0.0) {
    limits.max_linear_vel = max_linear;
  }
  if (max_angular > 0.0) {
    limits.max_angular_vel = max_angular;
  }

  session_->Begin(time_allowance_sec_, limits);
  AINFO << "[AssistedTeleopVelocity] session started (allowance="
        << time_allowance_sec_ << "s)";
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AssistedTeleopVelocityAction::onRunning()
{
  if (!session_ || !session_->IsActive()) {
    return BT::NodeStatus::SUCCESS;
  }
  if (isCancelRequested()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!session_->Tick([this]() { return isCancelRequested(); })) {
    session_->SetOutputCommand(commsgs::geometry_msgs::TwistStamped{});
    return BT::NodeStatus::SUCCESS;
  }

  auto ctx = taskContext();
  if (ctx) {
    navigator::teleop::VelocityFilterOptions filter_opts;
    filter_opts.projection_time_sec = projection_time_sec_;
    filter_opts.simulation_step_sec = simulation_step_sec_;
    filter_opts.disable_collision_checks = disable_collision_checks_;
    const auto safe = navigator::teleop::FilterVelocityForObstacles(
      *ctx, session_->RequestedCommand(), filter_opts);
    session_->SetOutputCommand(safe);
  }

  return BT::NodeStatus::RUNNING;
}

void AssistedTeleopVelocityAction::onHalted()
{
  if (session_) {
    session_->End();
  }
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<
    autonomy::tasks::behavior_tree::plugins::action::AssistedTeleopVelocityAction>(
    "AssistedTeleopVelocity");
}
