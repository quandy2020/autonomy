// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file runtime.hpp
 * @brief Native complete-coverage runtime implementation. */
#pragma once
#include "autonomy/task/navigation/coverage/coverage_visualizer.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/task/navigation/coverage/coverage_navigator.hpp"
#include "autonomy/task/navigation/coverage/tool_state.hpp"
#include "autonomy/task/scheduler/scheduler.hpp"
namespace autonomy::task::coverage {
class CoverageRuntime {
 public:
  CoverageRuntime(std::shared_ptr<autolink::Node>, TaskScheduler::SharedPtr,
                  navigation::NavigationClient::Ptr, Configuration = {},
                  std::string prefix = "");
  ~CoverageRuntime() { Shutdown(); }
  /** @brief Close admission and retire the owned execution resources. */
  void Shutdown();
  /** @brief Return the scheduler-registered coverage navigator. */
  std::shared_ptr<CoverageNavigator> Navigator() const { return navigator_; }

 private:
  std::shared_ptr<ToolState> tool_state_;
  std::shared_ptr<autolink::Writer<proto::ToolCommand>> tool_writer_;
  std::shared_ptr<autolink::Reader<proto::ToolAcknowledgment>> tool_reader_;
  std::shared_ptr<autolink::Writer<proto::CoverageDebug>> debug_writer_;
  std::shared_ptr<autolink::Writer<NavPath>> plan_writer_;
  std::shared_ptr<CoverageVisualizer> visualizer_;
  std::shared_ptr<CoverageServer> planner_;
  std::shared_ptr<CoverageNavigator> navigator_;
  std::unique_ptr<ActionEndpoint<ComputeAction>> planning_action_;
  std::unique_ptr<ActionEndpoint<NavigateAction>> navigation_action_;
};
}  // namespace autonomy::task::coverage
