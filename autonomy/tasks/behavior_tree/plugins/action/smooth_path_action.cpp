/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/node_utils.hpp"
#include "autonomy/planning/common/smoother_exceptions.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class SmoothPathAction : public BT::SyncActionNode
{
public:
    SmoothPathAction(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::planning_msgs::Path>("unsmoothed_path"),
            BT::OutputPort<commsgs::planning_msgs::Path>("smoothed_path"),
            BT::InputPort<std::string>("smoother_id", "", "Smoother id"),
            BT::InputPort<double>("max_smoothing_duration", 1.0, "Max seconds"),
            BT::InputPort<bool>("check_for_collisions", false, "Collision chk"),
        };
    }

    BT::NodeStatus tick() override {
        auto ctx = GetContext(config());
        if (!ctx || !ctx->smoother) {
            return BT::NodeStatus::FAILURE;
        }
        commsgs::planning_msgs::Path path;
        GetInputOrBB(*this, "unsmoothed_path", kBlackboardPathKey, path);
        const std::string smoother_id = ResolveSmootherId(*this, *ctx);
        double max_duration = 1.0;
        getInput("max_smoothing_duration", max_duration);
        bool check_collisions = false;
        getInput("check_for_collisions", check_collisions);
        try {
            auto result = ctx->smoother->SmoothPath(
                path, smoother_id,
                std::chrono::milliseconds(
                    static_cast<int>(max_duration * 1000.0)),
                check_collisions, ctx->CancelChecker());
            setOutput("smoothed_path", result.path);
            setOutput(kBlackboardPathKey, result.path);
            return BT::NodeStatus::SUCCESS;
        } catch (const planning::common::SmootherException&) {
            return BT::NodeStatus::FAILURE;
        }
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(SmoothPathAction, "SmoothPath")
