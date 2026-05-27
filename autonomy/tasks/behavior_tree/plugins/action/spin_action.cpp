/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/utils.hpp"
#include "autonomy/tasks/behavior_tree/recovery_action_node.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class SpinAction : public RecoveryActionNode
{
public:
    SpinAction(const std::string& name, const BT::NodeConfiguration& conf)
        : RecoveryActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("spin_dist", 1.57, "Yaw radians"),
            BT::InputPort<double>("time_allowance", 10.0, "Seconds"),
            BT::InputPort<bool>("is_recovery", false, "Recovery flag"),
        };
    }

protected:
    void onStart() override {
        auto ctx = Context();
        if (!ctx) {
            return;
        }
        double spin_dist = 1.57;
        double time_allowance = 10.0;
        bool is_recovery = false;
        getInput("spin_dist", spin_dist);
        getInput("time_allowance", time_allowance);
        getInput("is_recovery", is_recovery);
        if (is_recovery) {
            IncrementRecoveryCount(config());
        }
        control::ControllerServer::RecoveryMotionCommand cmd;
        cmd.type = control::ControllerServer::RecoveryMotionType::Spin;
        cmd.distance = spin_dist;
        cmd.speed = 0.5;
        cmd.time_allowance_sec = time_allowance;
        BeginRecovery(cmd);
    }

    BT::NodeStatus onRunning() override { return TickRecovery(); }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(SpinAction, "Spin")
