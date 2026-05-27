/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <chrono>

#include "autonomy/tasks/behavior_tree/action_node.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class WaitAction : public ActionNode
{
public:
    WaitAction(const std::string& name, const BT::NodeConfiguration& conf)
        : ActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("wait_duration", 1.0, "Seconds")};
    }

protected:
    void onStart() override {
        getInput("wait_duration", duration_sec_);
        deadline_ =
            std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(duration_sec_));
    }

    BT::NodeStatus onRunning() override {
        if (std::chrono::steady_clock::now() >= deadline_) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    double duration_sec_{1.0};
    std::chrono::steady_clock::time_point deadline_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "autonomy/tasks/behavior_tree/node_utils.hpp"

REGISTER_BEHAVIOR_TREE_NODE(WaitAction, "Wait")
