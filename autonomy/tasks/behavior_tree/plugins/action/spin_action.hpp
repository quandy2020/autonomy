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

class SpinAction : public BtStatefulActionNode
{
public:
    SpinAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("spin_dist", 1.57, "Spin distance"),
            BT::InputPort<double>("time_allowance", 10.0,
                                  "Allowed time for spinning"),
            BT::InputPort<bool>("is_recovery", true, "True if recovery"),
            BT::OutputPort<int32_t>("error_code_id", "Spin error code"),
            BT::OutputPort<std::string>("error_msg", "Spin error msg"),
        };
    }

private:
    bool started_motion_{false};
    bool is_recovery_{true};
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
