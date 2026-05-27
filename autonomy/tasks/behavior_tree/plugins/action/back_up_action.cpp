/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/utils.hpp"
#include "autonomy/tasks/behavior_tree/recovery_action_node.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class BackUpAction : public RecoveryActionNode
{
public:
    BackUpAction(const std::string& name, const BT::NodeConfiguration& conf)
        : RecoveryActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("backup_dist", 0.15, "Distance m"),
            BT::InputPort<double>("backup_speed", 0.1, "Speed m/s"),
            BT::InputPort<double>("time_allowance", 10.0, "Seconds"),
            BT::InputPort<bool>("disable_collision_checks", false, "No collide"),
        };
    }

protected:
    void onStart() override {
        auto ctx = Context();
        if (!ctx) {
            return;
        }
        double dist = 0.15;
        double speed = 0.1;
        double time_allowance = 10.0;
        bool disable_checks = false;
        getInput("backup_dist", dist);
        getInput("backup_speed", speed);
        getInput("time_allowance", time_allowance);
        getInput("disable_collision_checks", disable_checks);
        IncrementRecoveryCount(config());
        control::ControllerServer::RecoveryMotionCommand cmd;
        cmd.type = control::ControllerServer::RecoveryMotionType::BackUp;
        cmd.distance = dist;
        cmd.speed = speed;
        cmd.time_allowance_sec = time_allowance;
        cmd.disable_collision_checks = disable_checks;
        BeginRecovery(cmd);
    }

    BT::NodeStatus onRunning() override { return TickRecovery(); }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(BackUpAction, "BackUp")
