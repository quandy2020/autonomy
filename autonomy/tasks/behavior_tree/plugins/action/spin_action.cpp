/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller_server.hpp"
#include "autonomy/tasks/behavior_tree/bt_action_node.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace {

bool BuildSpinCommand(const BT::Blackboard::Ptr& blackboard,
                      const task_proto::SpinAction_Goal& goal,
                      control::ControllerServer::RecoveryMotionCommand& cmd) {
    commsgs::geometry_msgs::PoseStamped pose;
    if (!TryGetRobotPose(blackboard, pose)) {
        return false;
    }
    const double current_yaw =
        transform::tf2::getYaw(pose.pose.orientation);
    cmd.type = control::ControllerServer::RecoveryMotionType::Spin;
    cmd.distance = static_cast<double>(goal.target_yaw()) - current_yaw;
    cmd.speed = 0.5;
    cmd.time_allowance_sec =
        goal.has_time_allowance()
            ? commsgs::builtin_interfaces::FromProto(goal.time_allowance())
                  .Seconds()
            : 10.0;
    return true;
}

}  // namespace

class SpinAction : public BtActionNode<SpinActionTraits>
{
public:
    SpinAction(const std::string& name, const BT::NodeConfiguration& conf)
        : BtActionNode(name, kSpinActionName, conf) {}

    static BT::PortsList providedPorts() {
        return ProvidedBasicPorts({
            BT::InputPort<double>("spin_dist", 1.57, "Yaw radians"),
            BT::InputPort<double>("time_allowance", 10.0, "Seconds"),
            BT::InputPort<bool>("is_recovery", false, "Recovery flag"),
        });
    }

    BT::NodeStatus tick() override {
        const auto ctx = GetBtContext(config().blackboard);
        if (!ctx || !ctx->controller) {
            return BtActionNode::tick();
        }

        using TickResult = control::ControllerServer::RecoveryTickResult;

        if (!BT::isStatusActive(status())) {
            OnTick();
            if (!should_send_goal_) {
                return BT::NodeStatus::FAILURE;
            }
            control::ControllerServer::RecoveryMotionCommand cmd;
            if (!BuildSpinCommand(config().blackboard, goal_, cmd) ||
                !ctx->controller->BeginRecoveryMotion(cmd)) {
                return OnAborted();
            }
            in_process_active_ = true;
            setStatus(BT::NodeStatus::RUNNING);
        }

        if (WaitIfPaused(config())) {
            return BT::NodeStatus::RUNNING;
        }
        if (IsCancelRequested(config())) {
            ctx->controller->EndRecoveryMotion();
            in_process_active_ = false;
            return OnCancelled();
        }

        const auto tick_result =
            ctx->controller->TickRecoveryMotion(ctx->CancelChecker());
        if (tick_result == TickResult::Succeeded) {
            ctx->controller->EndRecoveryMotion();
            in_process_active_ = false;
            return OnSuccess();
        }
        if (tick_result == TickResult::Failed) {
            ctx->controller->EndRecoveryMotion();
            in_process_active_ = false;
            return OnAborted();
        }
        if (tick_result == TickResult::Cancelled) {
            ctx->controller->EndRecoveryMotion();
            in_process_active_ = false;
            return OnCancelled();
        }
        return BT::NodeStatus::RUNNING;
    }

    void halt() override {
        if (in_process_active_) {
            if (const auto ctx = GetBtContext(config().blackboard)) {
                if (ctx->controller) {
                    ctx->controller->EndRecoveryMotion();
                }
            }
            in_process_active_ = false;
        }
        BtActionNode::halt();
    }

    void OnTick() override {
        double spin_dist = 1.57;
        double time_allowance = 10.0;
        bool is_recovery = false;
        getInput("spin_dist", spin_dist);
        getInput("time_allowance", time_allowance);
        getInput("is_recovery", is_recovery);
        if (is_recovery) {
            IncrementRecoveryCount(config());
        }
        float target_yaw = static_cast<float>(spin_dist);
        commsgs::geometry_msgs::PoseStamped current_pose;
        if (TryGetRobotPose(config().blackboard, current_pose)) {
            target_yaw = static_cast<float>(
                transform::tf2::getYaw(current_pose.pose.orientation) +
                spin_dist);
        }
        goal_.set_target_yaw(target_yaw);
        *goal_.mutable_time_allowance() =
            commsgs::builtin_interfaces::ToProto(
                commsgs::builtin_interfaces::Duration::FromSeconds(
                    time_allowance));
    }

    BT::NodeStatus OnSuccess() override {
        setOutput("error_code_id", static_cast<uint16_t>(task_proto::SPIN_NONE));
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus OnAborted() override {
        setOutput("error_code_id",
                  static_cast<uint16_t>(task_proto::SPIN_UNKNOWN));
        return BT::NodeStatus::FAILURE;
    }

private:
    bool in_process_active_{false};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(SpinAction, "Spin")
