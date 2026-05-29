/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <cmath>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class TruncatePathAction : public BT::SyncActionNode
{
public:
    TruncatePathAction(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("distance", 1.0, "Distance before goal"),
            BT::InputPort<commsgs::planning_msgs::Path>("input_path"),
            BT::OutputPort<commsgs::planning_msgs::Path>("output_path"),
        };
    }

    BT::NodeStatus tick() override {
        commsgs::planning_msgs::Path path;
        if (!GetInputOrBlackboard(*this, config(), "input_path", kBlackboardPathKey,
                                  path) &&
            !getInput("input_path", path)) {
            return BT::NodeStatus::FAILURE;
        }
        if (path.poses.size() < 2) {
            setOutput("output_path", path);
            return BT::NodeStatus::SUCCESS;
        }
        double distance = 1.0;
        getInput("distance", distance);

        double accumulated = 0.0;
        size_t truncate_idx = path.poses.size() - 1;
        for (size_t i = path.poses.size() - 1; i > 0; --i) {
            const auto& p0 = path.poses[i];
            const auto& p1 = path.poses[i - 1];
            const double dx = p0.pose.position.x - p1.pose.position.x;
            const double dy = p0.pose.position.y - p1.pose.position.y;
            accumulated += std::hypot(dx, dy);
            if (accumulated >= distance) {
                truncate_idx = i - 1;
                break;
            }
        }
        commsgs::planning_msgs::Path out;
        out.header = path.header;
        out.poses.assign(path.poses.begin(),
                         path.poses.begin() +
                             static_cast<std::ptrdiff_t>(truncate_idx + 1));
        setOutput("output_path", out);
        setOutput(kBlackboardPathKey, out);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(TruncatePathAction, "TruncatePath")
