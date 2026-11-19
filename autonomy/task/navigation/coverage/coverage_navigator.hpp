// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_navigator.hpp
 * @brief Native complete-coverage coverage navigator implementation. */
#pragma once
#include "autonomy/task/common/motion_admission.hpp"
#include "autonomy/task/common/task_interface.hpp"
#include "autonomy/task/navigation/coverage/action_endpoint.hpp"
#include "autonomy/task/navigation/coverage/bt/coverage_nodes.hpp"
namespace autonomy::task::coverage {
class CoverageNavigator : public TaskInterface {
 public:
  CoverageNavigator(Configuration, std::shared_ptr<CoverageServer>,
                    navigation::NavigationClient::Ptr);
  ~CoverageNavigator() override { Shutdown(); }
  /** @brief Return the coverage task classification used by the scheduler. */
  automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType() const override;
  /** @brief Return the synchronized coverage task lifecycle. */
  TaskLifecycle GetLifecycle() const override;
  bool Initialize(const ::autonomy::task::proto::TaskServerOptions&) override;
  /** @brief Close admission and retire the owned execution resources. */
  void Shutdown() override;
  /** @brief Revoke further dispatch and request goal-specific cancellation. */
  bool Cancel() override;
  /** @brief Reject unsupported continuation without claiming work has paused.
   */
  bool Pause() override;
  /** @brief Reject unsupported continuation without repeating completed
   * coverage. */
  bool Resume() override;
  /** @brief Return identity, lifecycle and progress for scheduler polling. */
  ::autonomy::task::proto::ActiveTaskSnapshot GetSnapshot() const override;
  /** @brief Install the scheduler admission factory before execution. */
  void SetMotionAdmission(common::MotionAdmission::Factory);
  /** @brief Confirm execution and native cancellation evidence have drained. */
  bool MotionDrained() const;
  /** @brief Run the coverage behavior tree under one scheduler movement lease.
   */
  void Execute(
      const std::shared_ptr<ActionEndpoint<NavigateAction>::Handle>&,
      const CancelChecker&, NativeExecution::Tool,
      std::function<void(const ComputeAction::Result&)> plan_ready = {});

 private:
  Configuration config_;
  std::shared_ptr<CoverageServer> planner_;
  navigation::NavigationClient::Ptr navigation_;
  mutable std::mutex mutex_;
  TaskLifecycle lifecycle_{TaskLifecycle::kIdle};
  std::shared_ptr<common::MotionDispatchGate> gate_;
  common::MotionAdmission::Factory admission_;
  std::string task_id_, client_id_, tree_file_;
  double progress_{0};
  std::atomic<bool> stopping_{false}, canceling_{false}, executing_{false};
};
}  // namespace autonomy::task::coverage
