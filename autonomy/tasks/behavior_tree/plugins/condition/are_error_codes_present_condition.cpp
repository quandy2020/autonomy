/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <set>

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class AreErrorCodesPresentCondition : public BT::ConditionNode
{
public:
    AreErrorCodesPresentCondition(const std::string& name,
                                  const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<uint16_t>("error_code"),
            BT::InputPort<std::vector<int>>("error_codes_to_check"),
        };
    }

    BT::NodeStatus tick() override {
        uint16_t code = 0;
        std::vector<int> codes;
        getInput("error_code", code);
        if (!getInput("error_codes_to_check", codes)) {
            return BT::NodeStatus::FAILURE;
        }
        for (int c : codes) {
            if (static_cast<uint16_t>(c) == code) {
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::FAILURE;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(AreErrorCodesPresentCondition, "AreErrorCodesPresent")
