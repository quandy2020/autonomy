/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <chrono>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class PathExpiringTimerCondition : public BT::ConditionNode
{
public:
    PathExpiringTimerCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("seconds", 10.0, "Expiry seconds"),
            BT::InputPort<commsgs::planning_msgs::Path>("path"),
        };
    }

    BT::NodeStatus tick() override {
        if (first_time_) {
            getInput("seconds", period_sec_);
            GetInputOrBlackboard(*this, config(), "path", kBlackboardPathKey, prev_path_);
            start_ = std::chrono::steady_clock::now();
            first_time_ = false;
            return BT::NodeStatus::FAILURE;
        }
        commsgs::planning_msgs::Path path;
        GetInputOrBlackboard(*this, config(), "path", kBlackboardPathKey, path);
        if (!PathEqual(prev_path_, path)) {
            prev_path_ = path;
            start_ = std::chrono::steady_clock::now();
            return BT::NodeStatus::FAILURE;
        }
        const auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_);
        if (elapsed.count() < period_sec_) {
            return BT::NodeStatus::FAILURE;
        }
        start_ = std::chrono::steady_clock::now();
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool first_time_{true};
    double period_sec_{10.0};
    std::chrono::steady_clock::time_point start_;
    commsgs::planning_msgs::Path prev_path_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(PathExpiringTimerCondition, "PathExpiringTimer")
