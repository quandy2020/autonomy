/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class ArePosesNearCondition : public BT::ConditionNode
{
public:
    ArePosesNearCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("ref_pose"),
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("target_pose"),
            BT::InputPort<double>("tolerance", 0.25, "Meters"),
            BT::InputPort<std::string>("global_frame", "{global_frame}"),
        };
    }

    BT::NodeStatus tick() override {
        commsgs::geometry_msgs::PoseStamped ref_pose;
        commsgs::geometry_msgs::PoseStamped target_pose;
        double tolerance = 0.25;
        getInput("ref_pose", ref_pose);
        getInput("target_pose", target_pose);
        getInput("tolerance", tolerance);

        auto ctx = GetContext(config());
        if (!ctx || !ctx->tf_buffer) {
            return BT::NodeStatus::FAILURE;
        }
        std::string global_frame;
        getInput("global_frame", global_frame);
        if (global_frame.empty()) {
            global_frame = ctx->options.global_frame();
        }

        if (ref_pose.header.frame_id != target_pose.header.frame_id) {
            try {
                if (!ref_pose.header.frame_id.empty() &&
                    ref_pose.header.frame_id != global_frame) {
                    ref_pose = ctx->tf_buffer->transform(ref_pose, global_frame,
                                                         0.2f);
                }
                if (!target_pose.header.frame_id.empty() &&
                    target_pose.header.frame_id != global_frame) {
                    target_pose = ctx->tf_buffer->transform(
                        target_pose, global_frame, 0.2f);
                }
            } catch (const std::exception&) {
                return BT::NodeStatus::FAILURE;
            }
        }

        const double dx =
            ref_pose.pose.position.x - target_pose.pose.position.x;
        const double dy =
            ref_pose.pose.position.y - target_pose.pose.position.y;
        return (dx * dx + dy * dy) <= (tolerance * tolerance)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

private:
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(ArePosesNearCondition, "ArePosesNear")
