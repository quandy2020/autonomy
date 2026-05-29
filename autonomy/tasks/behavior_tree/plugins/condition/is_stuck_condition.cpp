/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <chrono>
#include <optional>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

namespace {
constexpr double kBrakeAccelLimit = -2.5;
}  // namespace

class IsStuckCondition : public BT::ConditionNode
{
public:
    IsStuckCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        auto ctx = GetContext(config());
        if (!ctx || !ctx->controller) {
            return BT::NodeStatus::FAILURE;
        }
        commsgs::planning_msgs::Odometry odom;
        if (!ctx->controller->GetLatestOdometry(odom)) {
            return BT::NodeStatus::FAILURE;
        }
        const auto now = std::chrono::steady_clock::now();
        if (last_odom_) {
            const double dt =
                std::chrono::duration<double>(now - last_time_).count();
            if (dt > 1e-3) {
                const double accel =
                    (odom.twist.twist.linear.x -
                     last_odom_->twist.twist.linear.x) /
                    dt;
                if (accel < kBrakeAccelLimit) {
                    return BT::NodeStatus::SUCCESS;
                }
            }
        }
        last_odom_ = odom;
        last_time_ = now;
        return BT::NodeStatus::FAILURE;
    }

private:
    std::optional<commsgs::planning_msgs::Odometry> last_odom_;
    std::chrono::steady_clock::time_point last_time_{std::chrono::steady_clock::now()};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(IsStuckCondition, "IsStuck")
