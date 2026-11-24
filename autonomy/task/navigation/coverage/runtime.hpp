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
 * @file runtime.hpp
 * @brief Native complete-coverage runtime implementation.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_RUNTIME_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_RUNTIME_HPP_

#include "autonomy/task/navigation/coverage/coverage_visualizer.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/task/navigation/coverage/coverage_navigator.hpp"
#include "autonomy/task/navigation/coverage/tool_state.hpp"
#include "autonomy/task/scheduler/scheduler.hpp"
namespace autonomy::task::coverage {
/**
 * @class autonomy::task::coverage::CoverageRuntime
 * @brief Owns every native object the coverage module needs at run time.
 *
 * Constructing the runtime registers the compute and navigate action servers,
 * opens the tool, debug and plan channels, and hands the navigator to the task
 * scheduler. Destruction retires all of it, so the module can be enabled or
 * disabled without leaving servers behind.
 */
class CoverageRuntime {
 public:
  /**
   * @brief Creates the endpoints, channels and the scheduler navigator.
   * @param[in] node Autolink node that hosts readers, writers and actions.
   * @param[in] scheduler Scheduler that receives the coverage navigator.
   * @param[in] client Navigation client used by the executor.
   * @param[in] configuration Coverage defaults for requests that omit values.
   * @param[in] prefix Channel prefix; empty uses the default @c /coverage name
   *            space.
   */
  CoverageRuntime(std::shared_ptr<autolink::Node> node,
                  TaskScheduler::SharedPtr scheduler,
                  navigation::NavigationClient::Ptr client,
                  Configuration configuration = {},
                  std::string prefix = "");

  /**
   * @brief Shuts the module down when the runtime leaves scope.
   */
  ~CoverageRuntime() { Shutdown(); }

  /**
   * @brief Closes admission and retires the owned execution resources.
   *
   * Safe to call twice: the endpoints are released on the first call and the
   * second call returns immediately.
   */
  void Shutdown();

  /**
   * @brief Accesses the scheduler-registered coverage navigator.
   * @return Navigator shared with the scheduler; null after @ref Shutdown.
   */
  std::shared_ptr<CoverageNavigator> Navigator() const { return navigator_; }

 private:
  // Tool command/acknowledgment handshake state.
  std::shared_ptr<ToolState> tool_state_;

  // Writes on/off tool commands.
  std::shared_ptr<autolink::Writer<proto::ToolCommand>> tool_writer_;

  // Receives tool acknowledgments from the driver.
  std::shared_ptr<autolink::Reader<proto::ToolAcknowledgment>> tool_reader_;

  // Publishes the per-goal planning stages.
  std::shared_ptr<autolink::Writer<proto::CoverageDebug>> debug_writer_;

  // Publishes the planned path for visualization.
  std::shared_ptr<autolink::Writer<NavPath>> plan_writer_;

  // Converts results into marker arrays.
  std::shared_ptr<CoverageVisualizer> visualizer_;

  // Stateless planner shared by every compute goal.
  std::shared_ptr<CoverageServer> planner_;

  // Navigator registered with the scheduler.
  std::shared_ptr<CoverageNavigator> navigator_;

  // Compute action endpoint (one goal at a time).
  std::unique_ptr<ActionEndpoint<ComputeAction>> planning_action_;

  // Navigate action endpoint (one goal at a time).
  std::unique_ptr<ActionEndpoint<NavigateAction>> navigation_action_;
};
}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_RUNTIME_HPP_
