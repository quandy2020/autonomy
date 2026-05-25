/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <string>

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/proto/bt_action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

class BackUpAction : public BtStatefulActionNode
{
public:
    BackUpAction(const std::string& xml_tag_name,
                 const BT::NodeConfiguration& conf);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("backup_dist", 0.15, "Distance to backup"),
            BT::InputPort<double>("backup_speed", 0.025, "Backup speed"),
            BT::InputPort<double>("time_allowance", 10.0, "Time allowance"),
            BT::InputPort<bool>("disable_collision_checks", false,
                                "Disable collision checking"),
            BT::OutputPort<int32_t>("error_code_id", "BackUp error code"),
            BT::OutputPort<std::string>("error_msg", "BackUp error msg"),
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
