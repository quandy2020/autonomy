// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_node.hpp
 * @brief Native coverage behavior-tree action. */
#pragma once
#include "autonomy/task/navigation/coverage/bt/coverage_nodes.hpp"
#include "autonomy/task/navigation/coverage/native_execution.hpp"
namespace autonomy::task::coverage {
template <int Stage>
class CoverageNode : public BT::StatefulActionNode {
 public:
  CoverageNode(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return {BT::OutputPort<NavPath>("nav_path"),
            BT::OutputPort<proto::PathComponents>("coverage_path"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
            BT::OutputPort<double>("planning_time")};
  }
  BT::NodeStatus onStart() override { return Poll(); }
  BT::NodeStatus onRunning() override { return Poll(); }
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
  std::shared_ptr<Execution> context_;
};

}  // namespace autonomy::task::coverage
