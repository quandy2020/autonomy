/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/planning/common/smoother_exceptions.hpp"
#include "autonomy/tasks/behavior_tree/bt_plugin_common.hpp"
#include "autonomy/tasks/behavior_tree/error_codes.hpp"
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
        getInput("unsmoothed_path", path);
        if (path.poses.empty()) {
            getInput(kBlackboardPathKey, path);
        }
        std::string smoother_id;
        getInput("smoother_id", smoother_id);
        if (smoother_id.empty()) {
            config().blackboard->get("selected_smoother", smoother_id);
        }
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

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::SmoothPathAction>(
        "SmoothPath");
}
