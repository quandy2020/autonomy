/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file coverage_navigator.hpp
 * @brief Native complete-coverage coverage navigator implementation.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_NAVIGATOR_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_NAVIGATOR_HPP_

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
  /**
   * @brief Return the coverage task classification used by the scheduler.
   */
  automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType() const override;
  /**
   * @brief Return the synchronized coverage task lifecycle.
   */
  TaskLifecycle GetLifecycle() const override;
  bool Initialize(const ::autonomy::task::proto::TaskServerOptions&) override;
  /**
   * @brief Close admission and retire the owned execution resources.
   */
  void Shutdown() override;
  /**
   * @brief Revoke further dispatch and request goal-specific cancellation.
   */
  bool Cancel() override;
  /** @brief Reject unsupported continuation without claiming work has paused.
   */
  bool Pause() override;
  /** @brief Reject unsupported continuation without repeating completed
   * coverage. */
  bool Resume() override;
  /**
   * @brief Return identity, lifecycle and progress for scheduler polling.
   */
  ::autonomy::task::proto::ActiveTaskSnapshot GetSnapshot() const override;
  /**
   * @brief Install the scheduler admission factory before execution.
   */
  void SetMotionAdmission(common::MotionAdmission::Factory);
  /**
   * @brief Confirm execution and native cancellation evidence have drained.
   */
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

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_NAVIGATOR_HPP_
