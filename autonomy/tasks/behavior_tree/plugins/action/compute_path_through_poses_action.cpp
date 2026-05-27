/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <vector>

#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/tasks/behavior_tree/bt_plugin_common.hpp"
#include "autonomy/tasks/behavior_tree/error_codes.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class ComputePathThroughPosesAction : public BT::SyncActionNode
{
public:
    ComputePathThroughPosesAction(const std::string& name,
                                  const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "goals"),
            BT::InputPort<std::string>("planner_id", "", "Planner id"),
            BT::OutputPort<commsgs::planning_msgs::Path>("path"),
            BT::OutputPort<uint16_t>("error_code_id"),
        };
    }

    BT::NodeStatus tick() override {
        auto ctx = GetContext(config());
        if (!ctx || !ctx->planner) {
            return BT::NodeStatus::FAILURE;
        }
        std::vector<commsgs::geometry_msgs::PoseStamped> goals;
        if (!getInput("goals", goals)) {
            getInput(kBlackboardGoalsKey, goals);
        }
        if (goals.empty()) {
            setOutput("error_code_id", BtErrorCode::INVALID_GOAL);
            return BT::NodeStatus::FAILURE;
        }
        std::string planner_id;
        getInput("planner_id", planner_id);
        if (planner_id.empty()) {
            config().blackboard->get("selected_planner", planner_id);
        }
        commsgs::geometry_msgs::PoseStamped start;
        if (!utils::getGlobalRobotPose(
                start, ctx->tf_buffer, ctx->controller->GetOdomSmoother(),
                ctx->options.global_frame(), ctx->options.robot_base_frame())) {
            setOutput("error_code_id", BtErrorCode::TF_ERROR);
            return BT::NodeStatus::FAILURE;
        }
        try {
            auto path = ctx->planner->ComputePathThroughPoses(
                start, goals, planner_id, ctx->CancelChecker());
            setOutput("path", path);
            setOutput(kBlackboardPathKey, path);
            setOutput("error_code_id", BtErrorCode::NONE);
            return BT::NodeStatus::SUCCESS;
        } catch (const planning::common::PlannerException&) {
            setOutput("error_code_id", BtErrorCode::PLANNER_FAILED);
            return BT::NodeStatus::FAILURE;
        }
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::ComputePathThroughPosesAction>(
        "ComputePathThroughPoses");
}
