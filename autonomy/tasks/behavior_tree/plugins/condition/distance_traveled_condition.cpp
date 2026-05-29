/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class DistanceTraveledCondition : public BT::ConditionNode
{
public:
    DistanceTraveledCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("distance", 1.0, "Meters"),
            BT::InputPort<std::string>("global_frame", "{global_frame}"),
            BT::InputPort<std::string>("robot_base_frame", "{robot_base_frame}"),
        };
    }

    BT::NodeStatus tick() override {
        if (!BT::isStatusActive(status())) {
            auto ctx = GetContext(config());
            if (!ctx || !GetGlobalStartPose(*ctx, start_pose_)) {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::FAILURE;
        }
        auto ctx = GetContext(config());
        if (!ctx) {
            return BT::NodeStatus::FAILURE;
        }
        commsgs::geometry_msgs::PoseStamped current;
        if (!GetGlobalStartPose(*ctx, current)) {
            return BT::NodeStatus::FAILURE;
        }
        double distance = 1.0;
        getInput("distance", distance);
        const double dx = current.pose.position.x - start_pose_.pose.position.x;
        const double dy = current.pose.position.y - start_pose_.pose.position.y;
        if (std::hypot(dx, dy) < distance) {
            return BT::NodeStatus::FAILURE;
        }
        start_pose_ = current;
        return BT::NodeStatus::SUCCESS;
    }

private:
    commsgs::geometry_msgs::PoseStamped start_pose_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(DistanceTraveledCondition, "DistanceTraveled")
