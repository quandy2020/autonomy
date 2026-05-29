/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/decorator_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class DistanceController : public BT::DecoratorNode
{
public:
    DistanceController(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::DecoratorNode(name, conf), first_time_(true) {}

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
            first_time_ = true;
        }

        setStatus(BT::NodeStatus::RUNNING);
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
        const double travelled = map::costmap_2d::utils::euclidean_distance(
            start_pose_.pose, current.pose);

        if (first_time_ || child_node_->status() == BT::NodeStatus::RUNNING ||
            travelled >= distance) {
            first_time_ = false;
            const BT::NodeStatus child_state = child_node_->executeTick();
            switch (child_state) {
                case BT::NodeStatus::SKIPPED:
                case BT::NodeStatus::RUNNING:
                    return child_state;
                case BT::NodeStatus::SUCCESS:
                    start_pose_ = current;
                    return BT::NodeStatus::SUCCESS;
                case BT::NodeStatus::FAILURE:
                default:
                    return BT::NodeStatus::FAILURE;
            }
        }
        return status();
    }

private:
    bool first_time_{true};
    commsgs::geometry_msgs::PoseStamped start_pose_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(DistanceController, "DistanceController")
