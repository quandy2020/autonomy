/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <chrono>
#include <optional>

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class IsStoppedCondition : public BT::ConditionNode
{
public:
    IsStoppedCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("velocity_threshold", 0.01, "m/s"),
            BT::InputPort<int>("duration_stopped", 1000, "ms"),
        };
    }

    BT::NodeStatus tick() override {
        auto ctx = GetContext(config());
        if (!ctx || !ctx->controller) {
            return BT::NodeStatus::FAILURE;
        }
        double threshold = 0.01;
        int duration_ms = 1000;
        getInput("velocity_threshold", threshold);
        getInput("duration_stopped", duration_ms);

        commsgs::planning_msgs::Odometry odom;
        if (!ctx->controller->GetLatestOdometry(odom)) {
            return BT::NodeStatus::FAILURE;
        }
        const auto& twist = odom.twist.twist;
        const bool stopped =
            std::abs(twist.linear.x) < threshold &&
            std::abs(twist.linear.y) < threshold &&
            std::abs(twist.angular.z) < threshold;
        const auto now = std::chrono::steady_clock::now();
        if (!stopped) {
            stopped_since_.reset();
            return BT::NodeStatus::FAILURE;
        }
        if (!stopped_since_) {
            stopped_since_ = now;
            return BT::NodeStatus::RUNNING;
        }
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                  *stopped_since_);
        if (elapsed_ms.count() >= duration_ms) {
            stopped_since_.reset();
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    std::optional<std::chrono::steady_clock::time_point> stopped_since_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(IsStoppedCondition, "IsStopped")
