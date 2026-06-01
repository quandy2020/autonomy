/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_action_node.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class ComputePathThroughPosesAction
    : public BtActionNode<ComputePathThroughPosesActionTraits>
{
public:
    ComputePathThroughPosesAction(const std::string& name,
                                  const BT::NodeConfiguration& conf)
        : BtActionNode(name, kComputePathThroughPosesActionName, conf) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = {
            BT::InputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "goals"),
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("start"),
            BT::InputPort<bool>("use_start", false, "Use port start pose"),
            BT::InputPort<std::string>("planner_id", "", "Planner id"),
            BT::OutputPort<commsgs::planning_msgs::Path>("path"),
        };
        return ProvidedBasicPorts(AppendErrorOutcomePorts(ports));
    }

    void OnTick() override {
        std::vector<commsgs::geometry_msgs::PoseStamped> goals;
        if (!GetInputOrBlackboard(*this, config(), "goals", kBlackboardGoalsKey,
                          goals) || goals.empty()) {
            should_send_goal_ = false;
            return;
        }
        goal_.mutable_goals()->clear_goals();
        for (const auto& g : goals) {
            *goal_.mutable_goals()->add_goals() =
                commsgs::geometry_msgs::ToProto(g);
        }
        getInput("planner_id", *goal_.mutable_planner_id());

        bool use_start = false;
        getInput("use_start", use_start);
        goal_.set_use_start(false);
        if (use_start) {
            commsgs::geometry_msgs::PoseStamped start;
            if (getInput("start", start)) {
                *goal_.mutable_start() = commsgs::geometry_msgs::ToProto(start);
                goal_.set_use_start(true);
            }
        } else if (getInput("start", *goal_.mutable_start())) {
            goal_.set_use_start(true);
        }
    }

    BT::NodeStatus OnSuccess() override {
        if (result_.result && result_.result->has_path()) {
            auto path = commsgs::planning_msgs::FromProto(result_.result->path());
            setOutput("path", path);
            setOutput(kBlackboardPathKey, path);
            NotifyGlobalPath(config().blackboard, path);
        }
        setOutput("error_code_id",
                  static_cast<uint16_t>(task_proto::COMPUTE_PATH_THROUGH_POSES_NONE));
        setOutput("error_msg", "");
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus OnAborted() override {
        commsgs::planning_msgs::Path empty;
        setOutput("path", empty);
        if (result_.result) {
            setOutput("error_code_id",
                      static_cast<uint16_t>(result_.result->error_code()));
            setOutput("error_msg", result_.result->error_msg());
        }
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus OnCancelled() override {
        commsgs::planning_msgs::Path empty;
        setOutput("path", empty);
        setOutput("error_code_id",
                  static_cast<uint16_t>(task_proto::COMPUTE_PATH_THROUGH_POSES_NONE));
        setOutput("error_msg", "");
        return BT::NodeStatus::SUCCESS;
    }

    void OnTimeout() override {
        setOutput("error_code_id",
                  static_cast<uint16_t>(task_proto::COMPUTE_PATH_THROUGH_POSES_TIMEOUT));
        setOutput("error_msg", "Behavior tree action client timed out.");
    }

    void halt() override {
        commsgs::planning_msgs::Path empty;
        setOutput("path", empty);
        BtActionNode::halt();
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(ComputePathThroughPosesAction,
                            "ComputePathThroughPoses")
