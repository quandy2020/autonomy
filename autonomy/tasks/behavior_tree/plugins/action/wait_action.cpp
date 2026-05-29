/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <chrono>

#include "autonomy/tasks/behavior_tree/bt_action_node.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class WaitAction : public BtActionNode<WaitActionTraits>
{
public:
    WaitAction(const std::string& name, const BT::NodeConfiguration& conf)
        : BtActionNode(name, kWaitActionName, conf) {}

    static BT::PortsList providedPorts() {
        return ProvidedBasicPorts(
            {BT::InputPort<double>("wait_duration", 1.0, "Seconds")});
    }

    BT::NodeStatus tick() override {
        if (!GetBtContext(config().blackboard)) {
            return BtActionNode::tick();
        }

        if (!BT::isStatusActive(status())) {
            OnTick();
            if (!should_send_goal_) {
                return BT::NodeStatus::FAILURE;
            }
            double duration_sec = 1.0;
            getInput("wait_duration", duration_sec);
            wait_deadline_ =
                std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(duration_sec));
            setStatus(BT::NodeStatus::RUNNING);
        }

        if (WaitIfPaused(config())) {
            return BT::NodeStatus::RUNNING;
        }
        if (IsCancelRequested(config())) {
            return OnCancelled();
        }
        if (std::chrono::steady_clock::now() >= wait_deadline_) {
            return OnSuccess();
        }
        return BT::NodeStatus::RUNNING;
    }

    void halt() override { resetStatus(); }

    void OnTick() override {
        double duration_sec = 1.0;
        getInput("wait_duration", duration_sec);
        *goal_.mutable_time() = commsgs::builtin_interfaces::ToProto(
            commsgs::builtin_interfaces::Duration::FromSeconds(duration_sec));
    }

private:
    std::chrono::steady_clock::time_point wait_deadline_{};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(WaitAction, "Wait")
