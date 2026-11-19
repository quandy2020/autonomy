// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_navigator.cpp
 * @brief Native complete-coverage coverage navigator implementation. */
#include "autonomy/task/navigation/coverage/coverage_navigator.hpp"

#include <filesystem>
#include <thread>

namespace autonomy::task::coverage {
CoverageNavigator::CoverageNavigator(Configuration c,
                                     std::shared_ptr<CoverageServer> planner,
                                     navigation::NavigationClient::Ptr nav)
    : config_(std::move(c)),
      planner_(std::move(planner)),
      navigation_(std::move(nav)) {}
automsgs::msgs::vehicle_msgs::RobotTaskType CoverageNavigator::GetTaskType()
    const {
  return automsgs::msgs::vehicle_msgs::ROBOT_TASK_COVERAGE;
}
TaskLifecycle CoverageNavigator::GetLifecycle() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return lifecycle_;
}
bool CoverageNavigator::Initialize(
    const ::autonomy::task::proto::TaskServerOptions& options) {
  if (executing_ || !MotionDrained()) return false;
  stopping_ = false;
  canceling_ = false;
  tree_file_ = std::string(AUTONOMY_COVERAGE_DEFAULT_TREE);
  const auto installed =
      std::filesystem::path(options.config_directory()) /
      "task/navigation/coverage/navigate_complete_coverage.xml";
  if (std::filesystem::exists(installed)) tree_file_ = installed.string();
  return std::filesystem::exists(tree_file_);
}
void CoverageNavigator::SetMotionAdmission(
    common::MotionAdmission::Factory factory) {
  std::lock_guard<std::mutex> lock(mutex_);
  admission_ = std::move(factory);
}
bool CoverageNavigator::MotionDrained() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return !executing_ && (!gate_ || gate_->Drained());
}
bool CoverageNavigator::Cancel() {
  canceling_ = true;
  std::shared_ptr<common::MotionDispatchGate> gate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gate = gate_;
  }
  if (gate) gate->Revoke();
  return true;
}
void CoverageNavigator::Shutdown() {
  stopping_ = true;
  Cancel();
}
// Restarting partly completed tool work needs an explicit continuation plan, so
// pause is rejected.
bool CoverageNavigator::Pause() { return false; }
bool CoverageNavigator::Resume() { return false; }
::autonomy::task::proto::ActiveTaskSnapshot CoverageNavigator::GetSnapshot()
    const {
  std::lock_guard<std::mutex> lock(mutex_);
  ::autonomy::task::proto::ActiveTaskSnapshot s;
  s.set_task_type(GetTaskType());
  s.set_task_status(ToRobotTaskStatus(lifecycle_));
  s.set_task_id(task_id_);
  s.set_client_id(client_id_);
  s.set_progress(progress_);
  return s;
}
void CoverageNavigator::Execute(
    const std::shared_ptr<ActionEndpoint<NavigateAction>::Handle>& handle,
    const CancelChecker& cancel, NativeExecution::Tool tool,
    std::function<void(const ComputeAction::Result&)> plan_ready) {
  const auto began = std::chrono::steady_clock::now();
  auto result = std::make_shared<NavigateAction::Result>();
  const auto goal = *handle->GetGoal();
  common::MotionAdmission::Factory factory;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    factory = admission_;
  }
  auto admission = factory ? factory() : nullptr;
  if (!admission || !admission->Valid() || stopping_) {
    result->set_error_code(proto::MOTION_BUSY);
    result->set_error_msg("Coverage has no scheduler movement admission");
    handle->Abort(result);
    return;
  }
  canceling_ = false;
  const auto permit = admission->PermitQuery();
  auto gate =
      std::make_shared<common::MotionDispatchGate>([this, permit, cancel] {
        return !stopping_ && !canceling_ && (!cancel || !cancel()) && permit &&
               permit();
      });
  if (!gate->Enable()) {
    admission->Complete(false);
    result->set_error_code(proto::MOTION_BUSY);
    handle->Abort(result);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gate_ = gate;
    lifecycle_ = TaskLifecycle::kRunning;
    task_id_ = goal.task_id();
    client_id_ = goal.client_id();
    progress_ = 0;
    executing_ = true;
  }
  admission->Complete(true);
  auto execution = std::make_shared<NativeExecution>(
      config_, planner_, navigation_, goal, gate, std::move(tool),
      std::move(plan_ready));
  const double timeout =
      goal.has_timeout_sec() ? goal.timeout_sec() : config_.navigation_timeout;
  BT::NodeStatus status = BT::NodeStatus::FAILURE;
  try {
    if (!std::isfinite(timeout) || timeout <= 0)
      throw RequestException("Navigation timeout must be positive and finite");
    BT::BehaviorTreeFactory bt;
    RegisterCoverageNodes(bt);
    auto board = BT::Blackboard::create();
    board->set<std::shared_ptr<Execution>>(kExecutionBlackboard, execution);
    auto tree = bt.createTreeFromFile(
        goal.behavior_tree().empty() ? tree_file_ : goal.behavior_tree(),
        board);
    for (;;) {
      if (stopping_ || canceling_ || (cancel && cancel())) {
        result->set_error_code(proto::CANCELED);
        result->set_error_msg("Coverage canceled");
        break;
      }
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - began)
                                 .count();
      if (elapsed >= timeout) {
        result->set_error_code(proto::TIMEOUT);
        result->set_error_msg("Coverage navigation timeout");
        break;
      }
      if (!gate->PermitValid()) {
        result->set_error_code(proto::CANCELED);
        result->set_error_msg("Scheduler movement permit revoked");
        break;
      }
      status = tree.tickOnce();
      auto feedback = std::make_shared<NavigateAction::Feedback>();
      feedback->set_navigation_time(elapsed);
      feedback->set_distance_remaining(execution->DistanceRemaining());
      feedback->set_progress(execution->ProgressValue());
      feedback->set_estimated_time_remaining(execution->DistanceRemaining() /
                                             config_.cruise_speed);
      feedback->set_stage(status == BT::NodeStatus::RUNNING ? "coverage"
                                                            : "terminal");
      handle->PublishFeedback(feedback);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        progress_ = execution->ProgressValue();
      }
      if (status != BT::NodeStatus::RUNNING) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    tree.haltTree();
    if (result->error_code() == proto::NONE &&
        status != BT::NodeStatus::SUCCESS) {
      result->set_error_code(execution->Error() == proto::NONE
                                 ? proto::UNKNOWN
                                 : execution->Error());
      result->set_error_msg(execution->ErrorMessage());
    }
  } catch (const std::exception& error) {
    result->set_error_code(proto::UNKNOWN);
    result->set_error_msg(error.what());
  }
  execution->Cancel();
  if (!gate->WaitForDrain(std::chrono::seconds(3))) {
    result->set_error_code(proto::TIMEOUT);
    result->set_error_msg(
        "Coverage cancellation lacks terminal movement evidence");
  }
  result->set_navigation_time(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
          .count());
  result->set_completed_segments(execution->CompletedSegments());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lifecycle_ = result->error_code() == proto::NONE ? TaskLifecycle::kSucceeded
                 : result->error_code() == proto::CANCELED
                     ? TaskLifecycle::kCanceled
                     : TaskLifecycle::kFailed;
    executing_ = false;
  }
  if (handle->IsCanceling())
    handle->Canceled(result);
  else if (result->error_code() == proto::NONE)
    handle->Succeed(result);
  else
    handle->Abort(result);
}
}  // namespace autonomy::task::coverage
