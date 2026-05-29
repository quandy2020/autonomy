/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <cmath>

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/tasks/behavior_tree/bt_action_node.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace {

bool SetRecoveryTargetFromDistance(const BT::NodeConfiguration& conf,
                                   double distance, bool backward,
                                   task_proto::BackUpAction_Goal& goal) {
    commsgs::geometry_msgs::PoseStamped pose;
    if (!TryGetRobotPose(conf.blackboard, pose)) {
        return false;
    }
    const double yaw = transform::tf2::getYaw(pose.pose.orientation);
    const double sign = backward ? -1.0 : 1.0;
    auto* target = goal.mutable_target();
    target->set_x(pose.pose.position.x + sign * std::cos(yaw) * distance);
    target->set_y(pose.pose.position.y + sign * std::sin(yaw) * distance);
    target->set_z(pose.pose.position.z);
    return true;
}

bool BuildBackUpCommand(
    const BT::Blackboard::Ptr& blackboard,
    const task_proto::BackUpAction_Goal& goal,
    control::ControllerServer::RecoveryMotionCommand& cmd) {
    commsgs::geometry_msgs::PoseStamped pose;
    if (!TryGetRobotPose(blackboard, pose) || !goal.has_target()) {
        return false;
    }
    const double dx = goal.target().x() - pose.pose.position.x;
    const double dy = goal.target().y() - pose.pose.position.y;
    cmd.type = control::ControllerServer::RecoveryMotionType::BackUp;
    cmd.distance = std::hypot(dx, dy);
    cmd.speed = goal.speed();
    cmd.time_allowance_sec =
        goal.has_time_allowance()
            ? commsgs::builtin_interfaces::FromProto(goal.time_allowance())
                  .Seconds()
            : 10.0;
    cmd.disable_collision_checks = goal.disable_collision_checks();
    return true;
}

}  // namespace

class BackUpAction : public BtActionNode<BackUpActionTraits>
{
public:
    BackUpAction(const std::string& name, const BT::NodeConfiguration& conf)
        : BtActionNode(name, kBackUpActionName, conf) {}

    static BT::PortsList providedPorts() {
        return ProvidedBasicPorts({
            BT::InputPort<double>("backup_dist", 0.15, "Distance m"),
            BT::InputPort<double>("backup_speed", 0.1, "Speed m/s"),
            BT::InputPort<double>("time_allowance", 10.0, "Seconds"),
            BT::InputPort<bool>("disable_collision_checks", false, "No collide"),
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
            if (!BuildBackUpCommand(config().blackboard, goal_, cmd) ||
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
        if (tick_result == TickResult::Failed ||
            tick_result == TickResult::Cancelled) {
            ctx->controller->EndRecoveryMotion();
            in_process_active_ = false;
            return OnAborted();
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

protected:
    void OnTick() override {
        double dist = 0.15;
        double speed = 0.1;
        double time_allowance = 10.0;
        bool disable_checks = false;
        getInput("backup_dist", dist);
        getInput("backup_speed", speed);
        getInput("time_allowance", time_allowance);
        getInput("disable_collision_checks", disable_checks);
        IncrementRecoveryCount(config());
        if (!SetRecoveryTargetFromDistance(config(), dist, true, goal_)) {
            should_send_goal_ = false;
            return;
        }
        goal_.set_speed(static_cast<float>(speed));
        goal_.set_disable_collision_checks(disable_checks);
        *goal_.mutable_time_allowance() =
            commsgs::builtin_interfaces::ToProto(
                commsgs::builtin_interfaces::Duration::FromSeconds(
                    time_allowance));
    }

    BT::NodeStatus OnSuccess() override {
        setOutput("error_code_id", static_cast<uint16_t>(task_proto::BACK_UP_NONE));
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus OnAborted() override {
        if (result_.result) {
            setOutput("error_code_id",
                      static_cast<uint16_t>(result_.result->error_code()));
        } else {
            setOutput("error_code_id",
                      static_cast<uint16_t>(task_proto::BACK_UP_UNKNOWN));
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    bool in_process_active_{false};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(BackUpAction, "BackUp")
