/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/recovery_action_node.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class DriveOnHeadingAction : public RecoveryActionNode
{
public:
    DriveOnHeadingAction(const std::string& name,
                         const BT::NodeConfiguration& conf)
        : RecoveryActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("dist_to_travel", 0.5, "Distance m"),
            BT::InputPort<double>("speed", 0.2, "Speed m/s"),
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
        double dist = 0.5;
        double speed = 0.2;
        double time_allowance = 10.0;
        bool disable_checks = false;
        getInput("dist_to_travel", dist);
        getInput("speed", speed);
        getInput("time_allowance", time_allowance);
        getInput("disable_collision_checks", disable_checks);
        control::ControllerServer::RecoveryMotionCommand cmd;
        cmd.type =
            control::ControllerServer::RecoveryMotionType::DriveOnHeading;
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

REGISTER_BEHAVIOR_TREE_NODE(DriveOnHeadingAction, "DriveOnHeading")
