/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/tasks/behavior_tree/bt_plugin_common.hpp"
#include "autonomy/tasks/behavior_tree/error_codes.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class ComputePathToPoseAction : public BT::SyncActionNode
{
public:
    ComputePathToPoseAction(const std::string& name,
                            const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("goal"),
            BT::InputPort<std::string>("planner_id", "", "Planner id"),
            BT::OutputPort<commsgs::planning_msgs::Path>("path"),
            BT::OutputPort<uint16_t>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

    BT::NodeStatus tick() override {
        auto ctx = GetContext(config());
        if (!ctx || !ctx->planner) {
            return BT::NodeStatus::FAILURE;
        }
        if (IsCancelRequested(config())) {
            return BT::NodeStatus::FAILURE;
        }
        commsgs::geometry_msgs::PoseStamped goal;
        if (!getInput("goal", goal)) {
            getInput(kBlackboardGoalKey, goal);
        }
        std::string planner_id;
        getInput("planner_id", planner_id);
        if (planner_id.empty()) {
            config().blackboard->get("selected_planner", planner_id);
        }
        if (planner_id.empty()) {
            planner_id = ctx->options.default_planner_id();
        }
        commsgs::geometry_msgs::PoseStamped start;
        if (!utils::getGlobalRobotPose(
                start, ctx->tf_buffer, ctx->controller->GetOdomSmoother(),
                ctx->options.global_frame(), ctx->options.robot_base_frame())) {
            setOutput("error_code_id", BtErrorCode::TF_ERROR);
            setOutput("error_msg", "Robot pose unavailable.");
            return BT::NodeStatus::FAILURE;
        }
        try {
            auto path = ctx->planner->ComputePathToPose(
                start, goal, planner_id, ctx->CancelChecker());
            setOutput("path", path);
            setOutput(kBlackboardPathKey, path);
            setOutput("error_code_id", BtErrorCode::NONE);
            setOutput("error_msg", std::string());
            return BT::NodeStatus::SUCCESS;
        } catch (const planning::common::PlannerException& ex) {
            setOutput("error_code_id", BtErrorCode::PLANNER_FAILED);
            setOutput("error_msg", ex.what());
            setOutput("path", commsgs::planning_msgs::Path{});
            return BT::NodeStatus::FAILURE;
        }
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::ComputePathToPoseAction>(
        "ComputePathToPose");
}
