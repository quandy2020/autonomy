/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class GetPoseFromPathAction : public BT::SyncActionNode
{
public:
    GetPoseFromPathAction(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::planning_msgs::Path>("path"),
            BT::InputPort<int>("index", -1, "Pose index, -1 = last"),
            BT::OutputPort<commsgs::geometry_msgs::PoseStamped>("pose"),
        };
    }

    BT::NodeStatus tick() override {
        commsgs::planning_msgs::Path path;
        if (!GetInputOrBlackboard(*this, config(), "path", kBlackboardPathKey,
                                  path)) {
            return BT::NodeStatus::FAILURE;
        }
        if (path.poses.empty()) {
            return BT::NodeStatus::FAILURE;
        }
        int index = -1;
        getInput("index", index);
        const size_t idx =
            index < 0 ? path.poses.size() - 1 : static_cast<size_t>(index);
        if (idx >= path.poses.size()) {
            return BT::NodeStatus::FAILURE;
        }
        setOutput("pose", path.poses[idx]);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(GetPoseFromPathAction, "GetPoseFromPath")
