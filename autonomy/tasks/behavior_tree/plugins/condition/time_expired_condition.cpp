/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <chrono>

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class TimeExpiredCondition : public BT::ConditionNode
{
public:
    TimeExpiredCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("seconds", 1.0, "Period seconds")};
    }

    BT::NodeStatus tick() override {
        if (!BT::isStatusActive(status())) {
            if (!getInput("seconds", period_sec_) || period_sec_ <= 0.0) {
                if (!config().blackboard->get(kBlackboardLocalSurvivalTimeoutKey,
                                               period_sec_) ||
                    period_sec_ <= 0.0) {
                    period_sec_ = 120.0;
                }
            }
            auto ctx = GetContext(config());
            start_ = ctx ? ctx->navigation_start
                         : std::chrono::steady_clock::now();
        }
        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - start_).count();
        if (elapsed < period_sec_) {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }

private:
    double period_sec_{120.0};
    std::chrono::steady_clock::time_point start_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(TimeExpiredCondition, "TimeExpired")
