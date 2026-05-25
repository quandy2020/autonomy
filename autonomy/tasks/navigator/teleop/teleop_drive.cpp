/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/tasks/navigator/teleop/teleop_drive.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/tasks/constants.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace teleop {

namespace {

double TimeAllowanceSec(const proto::AssistedTeleopAction::Goal & goal)
{
  if (!goal.has_time_allowance() || !goal.time_allowance().has_stamp()) {
    return 0.0;
  }
  const auto & stamp = goal.time_allowance().stamp();
  return static_cast<double>(stamp.sec()) +
    static_cast<double>(stamp.nanosec()) * 1e-9;
}

TeleopSession::Limits DefaultLimits(const proto::TaskOptions & options)
{
  TeleopSession::Limits limits;
  if (options.has_teleop_drive_options()) {
    const auto & opts = options.teleop_drive_options();
    limits.max_linear_vel = opts.default_max_linear_vel();
    limits.max_angular_vel = opts.default_max_angular_vel();
    limits.cmd_stale_timeout_sec = opts.cmd_stale_timeout_sec();
  }
  return limits;
}

}  // namespace

TeleopDriveNavigator::TeleopDriveNavigator(
  const proto::TaskOptions & options,
  const std::shared_ptr<common::TaskContext> & task_context,
  const std::vector<std::string> & plugin_lib_names,
  const common::FeedbackUtils & feedback_utils,
  const std::shared_ptr<common::NavigatorMuxer> & muxer,
  std::shared_ptr<common::OdomSmoother> odom_smoother)
: BtNavigator<ActionT>(
    kNavigatorTeleopDrive, "teleop_drive.xml", options, task_context,
    plugin_lib_names, feedback_utils, muxer, odom_smoother),
  task_context_(task_context),
  options_(options),
  teleop_session_(std::make_shared<TeleopSession>())
{
  if (auto blackboard = GetBlackboard()) {
    blackboard->set("teleop_session", teleop_session_);  // NOLINT
  }
}

void TeleopDriveNavigator::setRunLimits(
  const double max_linear_vel, const double max_angular_vel)
{
  run_max_linear_vel_ = max_linear_vel;
  run_max_angular_vel_ = max_angular_vel;
}

std::shared_ptr<TeleopSession> TeleopDriveNavigator::session() const
{
  return teleop_session_;
}

bool TeleopDriveNavigator::GoalReceived(
  std::shared_ptr<const typename ActionT::Goal> goal)
{
  if (!goal) {
    return false;
  }

  std::string bt_xml = GetDefaultBTFilename();
  bt_xml = common::ResolveBehaviorTreeFile(bt_xml, feedback_utils_);
  if (!LoadBehaviorTree(bt_xml)) {
    SetInternalError(
      static_cast<uint16_t>(
        proto::ASSISTED_TELEOP_ERROR_UNKNOWN),
      "Failed to load teleop behavior tree: " + bt_xml);
    return false;
  }

  auto limits = DefaultLimits(options_);
  if (run_max_linear_vel_ > 0.0) {
    limits.max_linear_vel = run_max_linear_vel_;
  }
  if (run_max_angular_vel_ > 0.0) {
    limits.max_angular_vel = run_max_angular_vel_;
  }

  start_time_ = std::chrono::steady_clock::now();
  if (auto blackboard = GetBlackboard()) {
    const int mode = static_cast<int>(goal->mode());

    double linear_speed = goal->linear_speed();
    double angular_speed = goal->angular_speed();
    if (linear_speed <= 0.0) {
      linear_speed = limits.max_linear_vel;
    }
    if (angular_speed <= 0.0) {
      angular_speed = limits.max_angular_vel;
    }

    double projection_time = 1.5;
    double simulation_step = 0.1;
    if (options_.has_teleop_drive_options()) {
      const auto & opts = options_.teleop_drive_options();
      if (opts.projection_time_sec() > 0.0) {
        projection_time = opts.projection_time_sec();
      }
      if (opts.simulation_step_sec() > 0.0) {
        simulation_step = opts.simulation_step_sec();
      }
    }

    blackboard->set("number_recoveries", 0);  // NOLINT
    blackboard->set("initial_pose_received", true);  // NOLINT
    blackboard->set("teleop_mode", mode);  // NOLINT
    blackboard->set("teleop_time_allowance", TimeAllowanceSec(*goal));  // NOLINT
    blackboard->set("teleop_max_linear_vel", limits.max_linear_vel);    // NOLINT
    blackboard->set("teleop_max_angular_vel", limits.max_angular_vel);  // NOLINT
    blackboard->set("teleop_cmd_stale_timeout_sec",  // NOLINT
                    limits.cmd_stale_timeout_sec);
    blackboard->set("teleop_linear_distance",  // NOLINT
                    std::abs(static_cast<double>(goal->linear_distance())));
    blackboard->set("teleop_linear_signed",  // NOLINT
                    static_cast<double>(goal->linear_distance()));
    blackboard->set("teleop_linear_speed", linear_speed);  // NOLINT
    blackboard->set("teleop_rotation_angle",  // NOLINT
                    static_cast<double>(goal->rotation_angle()));
    blackboard->set("teleop_angular_speed", angular_speed);  // NOLINT
    blackboard->set("teleop_disable_collision_checks",  // NOLINT
                    goal->disable_collision_checks());
    blackboard->set("teleop_projection_time_sec", projection_time);  // NOLINT
    blackboard->set("teleop_simulation_step_sec", simulation_step);  // NOLINT
  }
  return true;
}

void TeleopDriveNavigator::GoalCompleted(
  std::shared_ptr<typename ActionT::Result> result,
  const common::BtStatus final_bt_status)
{
  if (auto teleop = session()) {
    teleop->End();
  }
  if (!result) {
    return;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();
  *result->mutable_total_elapsed_time() =
    commsgs::builtin_interfaces::ToProto(
      commsgs::builtin_interfaces::Duration::FromSeconds(elapsed));

  if (final_bt_status == common::BtStatus::SUCCEEDED) {
    result->set_error_code(
      proto::ASSISTED_TELEOP_ERROR_NONE);
  } else if (final_bt_status == common::BtStatus::CANCELED) {
    result->set_error_code(
      proto::ASSISTED_TELEOP_ERROR_NONE);
  } else {
    result->set_error_code(
      proto::ASSISTED_TELEOP_ERROR_UNKNOWN);
    result->set_error_msg("Teleop behavior tree failed");
  }
}

void TeleopDriveNavigator::OnLoop()
{
  auto feedback = std::make_shared<typename ActionT::Feedback>();
  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_);
  *feedback->mutable_current_teleop_duration() =
    commsgs::builtin_interfaces::ToProto(
      commsgs::builtin_interfaces::Duration::FromSeconds(elapsed.count()));
  if (auto teleop = session()) {
    feedback->set_commanded_linear_vel(
      static_cast<float>(teleop->commandedLinearVel()));
    feedback->set_commanded_angular_vel(
      static_cast<float>(teleop->commandedAngularVel()));
    feedback->set_applied_linear_vel(
      static_cast<float>(teleop->appliedLinearVel()));
    feedback->set_applied_angular_vel(
      static_cast<float>(teleop->appliedAngularVel()));
  }
  PublishFeedback(feedback);
}

void TeleopDriveNavigator::OnPreempt(
  std::shared_ptr<const typename ActionT::Goal> goal)
{
  if (goal && GoalReceived(goal)) {
    AcceptPendingGoal();
  } else {
    TerminatePendingGoal();
  }
}

}  // namespace teleop
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
