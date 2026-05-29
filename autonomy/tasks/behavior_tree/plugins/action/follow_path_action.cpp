/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller_server.hpp"
#include "autonomy/tasks/behavior_tree/bt_action_node.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class FollowPathAction : public BtActionNode<FollowPathActionTraits>
{
public:
    FollowPathAction(const std::string& name, const BT::NodeConfiguration& conf)
        : BtActionNode(name, kFollowPathActionName, conf) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = {
            BT::InputPort<commsgs::planning_msgs::Path>("path"),
            BT::InputPort<std::string>("controller_id", "", "Controller"),
            BT::InputPort<std::string>("goal_checker_id", "", "Goal checker"),
            BT::InputPort<std::string>("progress_checker_id", "",
                                       "Progress checker"),
        };
        return ProvidedBasicPorts(AppendErrorOutcomePorts(ports));
    }

    BT::NodeStatus tick() override {
        const auto ctx = GetBtContext(config().blackboard);
        if (!ctx || !ctx->controller) {
            return BtActionNode::tick();
        }

        using TickResult = control::ControllerServer::FollowPathTickResult;

        if (!BT::isStatusActive(status())) {
            OnTick();
            if (!should_send_goal_) {
                return BT::NodeStatus::FAILURE;
            }
            const auto path = commsgs::planning_msgs::FromProto(goal_.path());
            std::string goal_checker_id;
            std::string progress_checker_id;
            getInput("goal_checker_id", goal_checker_id);
            getInput("progress_checker_id", progress_checker_id);
            if (!ctx->controller->BeginFollowPath(
                    path, goal_.controller_id(), goal_checker_id,
                    progress_checker_id)) {
                return OnAborted();
            }
            in_process_active_ = true;
            setStatus(BT::NodeStatus::RUNNING);
        }

        if (WaitIfPaused(config())) {
            return BT::NodeStatus::RUNNING;
        }

        if (IsCancelRequested(config())) {
            ctx->controller->EndFollowPath();
            in_process_active_ = false;
            return OnCancelled();
        }

        const auto tick_result =
            ctx->controller->TickFollowPath(ctx->CancelChecker());
        if (tick_result == TickResult::Succeeded) {
            ctx->controller->EndFollowPath();
            in_process_active_ = false;
            return OnSuccess();
        }
        if (tick_result == TickResult::Failed) {
            ctx->controller->EndFollowPath();
            in_process_active_ = false;
            return OnAborted();
        }
        if (tick_result == TickResult::Cancelled) {
            ctx->controller->EndFollowPath();
            in_process_active_ = false;
            return OnCancelled();
        }
        return BT::NodeStatus::RUNNING;
    }

    void halt() override {
        if (in_process_active_) {
            if (const auto ctx = GetBtContext(config().blackboard)) {
                if (ctx->controller) {
                    ctx->controller->EndFollowPath();
                }
            }
            in_process_active_ = false;
        }
        BtActionNode::halt();
    }

    void OnTick() override {
        commsgs::planning_msgs::Path path;
        if (!GetInputOrBlackboard(*this, config(), "path", kBlackboardPathKey,
                                  path)) {
            should_send_goal_ = false;
            return;
        }
        *goal_.mutable_path() = commsgs::planning_msgs::ToProto(path);
        getInput("controller_id", *goal_.mutable_controller_id());
        getInput("goal_checker_id", *goal_.mutable_goal_checker_id());
        getInput("progress_checker_id", *goal_.mutable_progress_checker_id());
    }

    BT::NodeStatus OnSuccess() override {
        setOutput("error_code_id",
                  static_cast<uint16_t>(task_proto::FOLLOW_PATH_NONE));
        setOutput("error_msg", "");
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus OnAborted() override {
        setOutput("error_code_id",
                  static_cast<uint16_t>(task_proto::FOLLOW_PATH_UNKNOWN));
        setOutput("error_msg", "FollowPath failed in-process.");
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus OnCancelled() override {
        setOutput("error_code_id",
                  static_cast<uint16_t>(task_proto::FOLLOW_PATH_NONE));
        setOutput("error_msg", "");
        return BT::NodeStatus::SUCCESS;
    }

    void OnTimeout() override {
        setOutput("error_code_id",
                  static_cast<uint16_t>(task_proto::FOLLOW_PATH_CONTROLLER_TIMED_OUT));
        setOutput("error_msg", "Behavior tree action client timed out.");
    }

private:
    bool in_process_active_{false};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(FollowPathAction, "FollowPath")
