/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <string>

#include "autonomy/tasks/behavior_tree/bt_stateful_action_node.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

class DriveOnHeadingAction : public BtStatefulActionNode
{
public:
    DriveOnHeadingAction(const std::string& xml_tag_name,
                         const BT::NodeConfiguration& conf);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("dist_to_travel", 0.15, "Distance to travel"),
            BT::InputPort<double>("speed", 0.025, "Travel speed"),
            BT::InputPort<double>("time_allowance", 10.0, "Time allowance"),
            BT::InputPort<bool>("disable_collision_checks", false,
                                "Disable collision checking"),
            BT::OutputPort<int32_t>("error_code_id", "DriveOnHeading error code"),
            BT::OutputPort<std::string>("error_msg", "DriveOnHeading error msg"),
        };
    }

private:
    bool started_motion_{false};
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
