/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"
#include <set>

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class WouldAControllerRecoveryHelpCondition : public BT::ConditionNode
{
public:
    WouldAControllerRecoveryHelpCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<uint16_t>("error_code")};
    }

    BT::NodeStatus tick() override {
        uint16_t code = 0;
        getInput("error_code", code);
        static const std::set<uint16_t> kControllerErrors = {
            ExePathOutcome::FAILURE, ExePathOutcome::PAT_EXCEEDED,
            ExePathOutcome::NO_VALID_CMD, ExePathOutcome::ROBOT_STUCK,
            ExePathOutcome::OSCILLATION, ExePathOutcome::MISSED_PATH,
            ExePathOutcome::BLOCKED_PATH, ExePathOutcome::INVALID_PATH};
        return kControllerErrors.count(code) ? BT::NodeStatus::SUCCESS
                                           : BT::NodeStatus::FAILURE;
    }

private:
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(WouldAControllerRecoveryHelpCondition,
                            "WouldAControllerRecoveryHelp")
