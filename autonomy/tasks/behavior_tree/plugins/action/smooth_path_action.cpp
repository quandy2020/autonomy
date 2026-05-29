/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autolink/action/types.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_action_node.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class SmoothPathAction : public BtActionNode<SmoothPathActionTraits>
{
public:
    SmoothPathAction(const std::string& name, const BT::NodeConfiguration& conf)
        : BtActionNode(name, kSmoothPathActionName, conf) {}

    static BT::PortsList providedPorts() {
        return ProvidedBasicPorts({
            BT::InputPort<commsgs::planning_msgs::Path>("unsmoothed_path"),
            BT::OutputPort<commsgs::planning_msgs::Path>("smoothed_path"),
            BT::InputPort<std::string>("smoother_id", "", "Smoother id"),
            BT::InputPort<double>("max_smoothing_duration", 1.0, "Max seconds"),
            BT::InputPort<bool>("check_for_collisions", false, "Collision chk"),
        });
    }

    bool ExecuteInProcess(WrappedResult& result) override {
        const auto ctx = GetBtContext(config().blackboard);
        if (!ctx || !ctx->smoother) {
            return false;
        }
        commsgs::planning_msgs::Path path =
            commsgs::planning_msgs::FromProto(goal_.path());
        std::chrono::milliseconds max_time{1000};
        if (goal_.has_max_smoothing_duration()) {
            const double max_sec =
                commsgs::builtin_interfaces::FromProto(
                    goal_.max_smoothing_duration())
                    .Seconds();
            if (max_sec > 0.0) {
                max_time = std::chrono::milliseconds(static_cast<int>(
                    max_sec * 1000.0));
            }
        }
        try {
            const auto smooth_result = ctx->smoother->SmoothPath(
                path, goal_.smoother_id(), max_time, goal_.check_for_collisions(),
                ctx->CancelChecker());
            result.code = autolink::action::ResultCode::SUCCEEDED;
            result.result = std::make_shared<Result>();
            *result.result->mutable_path() =
                commsgs::planning_msgs::ToProto(smooth_result.path);
            result.result->set_error_code(task_proto::SMOOTH_PATH_NONE);
            NotifyGlobalPath(config().blackboard, smooth_result.path);
            return true;
        } catch (const std::exception& ex) {
            result.code = autolink::action::ResultCode::ABORTED;
            result.result = std::make_shared<Result>();
            result.result->set_error_code(task_proto::SMOOTH_PATH_UNKNOWN);
            result.result->set_error_msg(ex.what());
            return true;
        }
    }

    void OnTick() override {
        commsgs::planning_msgs::Path path;
        if (!GetInputOrBlackboard(*this, config(), "unsmoothed_path", kBlackboardPathKey,
                          path)) {
            should_send_goal_ = false;
            return;
        }
        *goal_.mutable_path() = commsgs::planning_msgs::ToProto(path);
        std::string smoother_id;
        if (getInput("smoother_id", smoother_id)) {
            goal_.set_smoother_id(smoother_id);
        }
        double max_duration = 1.0;
        getInput("max_smoothing_duration", max_duration);
        *goal_.mutable_max_smoothing_duration() =
            commsgs::builtin_interfaces::ToProto(
                commsgs::builtin_interfaces::Duration::FromSeconds(max_duration));
        bool check_for_collisions = false;
        getInput("check_for_collisions", check_for_collisions);
        goal_.set_check_for_collisions(check_for_collisions);
    }

    BT::NodeStatus OnSuccess() override {
        if (result_.result && result_.result->has_path()) {
            auto path = commsgs::planning_msgs::FromProto(result_.result->path());
            setOutput("smoothed_path", path);
            setOutput(kBlackboardPathKey, path);
            NotifyGlobalPath(config().blackboard, path);
        }
        setOutput("error_code_id", static_cast<uint16_t>(task_proto::SMOOTH_PATH_NONE));
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus OnAborted() override {
        return BT::NodeStatus::FAILURE;
    }

};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(SmoothPathAction, "SmoothPath")
