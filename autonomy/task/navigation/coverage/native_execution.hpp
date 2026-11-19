// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file native_execution.hpp
 * @brief NativeExecution coverage execution contract. */
#pragma once
#include "autonomy/task/navigation/action_session.hpp"
#include "autonomy/task/navigation/coverage/coverage_server.hpp"
#include "autonomy/task/navigation/coverage/execution.hpp"
#include "autonomy/task/navigation/coverage/segment.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
namespace autonomy::task::coverage {
/** @brief NativeExecution execution lifecycle and task actions. */
class NativeExecution : public Execution {
 public:
  using Tool = std::function<bool(bool, double)>;
  NativeExecution(
      Configuration, std::shared_ptr<CoverageServer>,
      navigation::NavigationClient::Ptr, NavigateAction::Goal,
      std::shared_ptr<common::MotionDispatchGate>, Tool,
      std::function<void(const ComputeAction::Result&)> plan_ready = {});
  /** @brief Execute the requested coverage planning pipeline. */
  BT::NodeStatus Compute() override;
  /** @brief Reach the first coverage pose through the native navigation
   * planner. */
  BT::NodeStatus NavigateToStart() override;
  /** @brief Advance only after the current coverage segment terminates
   * successfully. */
  BT::NodeStatus FollowCoverage() override;
  /** @brief Revoke further dispatch and request goal-specific cancellation. */
  void Cancel() override;
  /** @brief Return the sum of remaining coverage segment lengths. */
  double DistanceRemaining() const;
  /** @brief Return completed coverage distance as a fraction. */
  double ProgressValue() const;
  /** @brief Return the number of successful coverage segments. */
  uint32_t CompletedSegments() const { return static_cast<uint32_t>(index_); }
  /** @brief Return the current domain error code. */
  proto::ErrorCode Error() const { return error_; }
  const std::string& ErrorMessage() const { return message_; }
  const ComputeAction::Result& Plan() const { return plan_; }

 private:
  /** @brief Send and observe one bounded path under the retained motion gate.
   */
  BT::NodeStatus Follow(const Segment&);
  /** @brief Cancel execution and retain a diagnostic task failure. */
  BT::NodeStatus Fail(proto::ErrorCode, const std::string&);
  Configuration config_;
  std::shared_ptr<CoverageServer> planner_;
  navigation::NavigationClient::Ptr navigation_;
  NavigateAction::Goal goal_;
  std::shared_ptr<common::MotionDispatchGate> gate_;
  Tool tool_;
  ComputeAction::Result plan_;
  std::vector<Segment> segments_;
  size_t index_{0};
  std::function<void(const ComputeAction::Result&)> plan_ready_;
  bool computed_{false}, sent_{false}, approach_planned_{false},
      approached_{false};
  std::vector<Segment> approach_;
  size_t approach_index_{0};
  navigation::ActionSession<automsgs::actions::FollowPathAction> motion_;
  proto::ErrorCode error_{proto::NONE};
  std::string message_;
  double total_length_{0};
};
}  // namespace autonomy::task::coverage
