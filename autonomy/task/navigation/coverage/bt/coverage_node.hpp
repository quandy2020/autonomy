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
 * @file coverage_node.hpp
 * @brief Native coverage behavior-tree action.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_BT_COVERAGE_NODE_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_BT_COVERAGE_NODE_HPP_

#include "autonomy/task/navigation/coverage/bt/coverage_nodes.hpp"
#include "autonomy/task/navigation/coverage/native_execution.hpp"
namespace autonomy::task::coverage {
/**
 * @class autonomy::task::coverage::CoverageNode
 * @brief Behavior tree node that drives one coverage stage to completion.
 *
 * The node is stateful: the first tick starts the stage and every following tick
 * polls it, so a long plan or a long drive does not block the tree. Halted nodes
 * cancel the stage through the execution object.
 *
 * @tparam Stage 0 plans, 1 drives to the start pose, 2 follows the coverage.
 */
template <int Stage>
class CoverageNode : public BT::StatefulActionNode {
 public:
  /**
   * @brief Constructs the node from the tree node configuration.
   * @param[in] name Node name from the tree XML.
   * @param[in] config Ports and blackboard supplied by the tree.
   */
  CoverageNode(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

  /**
   * @brief Declares the result ports of the node.
   * @return Ports exported to the tree: @c nav_path, @c coverage_path,
   *         @c error_code_id, @c error_msg and @c planning_time.
   */
  static BT::PortsList providedPorts() {
    return {BT::OutputPort<NavPath>("nav_path"),
            BT::OutputPort<proto::PathComponents>("coverage_path"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
            BT::OutputPort<double>("planning_time")};
  }
  /**
   * @brief Starts the stage on the first tick.
   * @return Stage status after the first poll.
   */
  BT::NodeStatus onStart() override { return Poll(); }

  /**
   * @brief Polls the stage while the tree keeps ticking.
   * @return Stage status; RUNNING keeps the node active.
   */
  BT::NodeStatus onRunning() override { return Poll(); }

  /**
   * @brief Cancels the stage when the tree is halted or preempted.
   */
  void onHalted() override {
    if (context_) context_->Cancel();
  }

 private:
  BT::NodeStatus Poll() {
    if (!context_)
      context_ = config().blackboard->get<std::shared_ptr<Execution>>(
          kExecutionBlackboard);
    const auto status = Stage == 0   ? context_->Compute()
                        : Stage == 1 ? context_->NavigateToStart()
                                     : context_->FollowCoverage();
    if (auto native = std::dynamic_pointer_cast<NativeExecution>(context_)) {
      setOutput("error_code_id", static_cast<int>(native->Error()));
      setOutput("error_msg", native->ErrorMessage());
      if (Stage == 0 && status == BT::NodeStatus::SUCCESS) {
        setOutput("nav_path", native->Plan().nav_path());
        setOutput("coverage_path", native->Plan().coverage_path());
        setOutput("planning_time", native->Plan().planning_time());
      }
    }
    return status;
  }
  // Execution shared with the sibling stage nodes (resolved on first tick).
  std::shared_ptr<Execution> context_;
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_BT_COVERAGE_NODE_HPP_
