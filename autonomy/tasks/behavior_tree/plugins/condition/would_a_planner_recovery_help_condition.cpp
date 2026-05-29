/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"
#include <set>

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class WouldAPlannerRecoveryHelpCondition : public BT::ConditionNode
{
public:
    WouldAPlannerRecoveryHelpCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<uint16_t>("error_code")};
    }

    BT::NodeStatus tick() override {
        uint16_t code = 0;
        getInput("error_code", code);
        static const std::set<uint16_t> kPlannerErrors = {
            GetPathOutcome::FAILURE, GetPathOutcome::NO_PATH_FOUND,
            GetPathOutcome::PAT_EXCEEDED, GetPathOutcome::TF_ERROR,
            GetPathOutcome::INVALID_PLUGIN, GetPathOutcome::EMPTY_PATH};
        return kPlannerErrors.count(code) ? BT::NodeStatus::SUCCESS
                                          : BT::NodeStatus::FAILURE;
    }

private:
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(WouldAPlannerRecoveryHelpCondition,
                            "WouldAPlannerRecoveryHelp")
