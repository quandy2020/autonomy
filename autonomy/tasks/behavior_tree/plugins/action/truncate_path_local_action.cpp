/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <cmath>
#include <limits>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class TruncatePathLocalAction : public BT::SyncActionNode
{
public:
    TruncatePathLocalAction(const std::string& name,
                            const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::planning_msgs::Path>("input_path"),
            BT::InputPort<double>("distance_forward", 8.0, "Forward m"),
            BT::InputPort<double>("distance_backward", 2.0, "Backward m"),
            BT::InputPort<std::string>("robot_frame", "base_link", "Base frame"),
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
        if (path.poses.empty()) {
            return BT::NodeStatus::FAILURE;
        }

        auto ctx = GetContext(config());
        commsgs::geometry_msgs::PoseStamped robot;
        if (!ctx || !GetGlobalStartPose(*ctx, robot)) {
            return BT::NodeStatus::FAILURE;
        }

        double forward = 8.0;
        double backward = 2.0;
        getInput("distance_forward", forward);
        getInput("distance_backward", backward);

        size_t closest = 0;
        double best = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path.poses.size(); ++i) {
            const double dx =
                path.poses[i].pose.position.x - robot.pose.position.x;
            const double dy =
                path.poses[i].pose.position.y - robot.pose.position.y;
            const double d = dx * dx + dy * dy;
            if (d < best) {
                best = d;
                closest = i;
            }
        }

        double acc_fwd = 0.0;
        size_t end_idx = path.poses.size() - 1;
        for (size_t i = closest; i + 1 < path.poses.size(); ++i) {
            const auto& a = path.poses[i];
            const auto& b = path.poses[i + 1];
            acc_fwd += std::hypot(b.pose.position.x - a.pose.position.x,
                                b.pose.position.y - a.pose.position.y);
            end_idx = i + 1;
            if (acc_fwd >= forward) {
                break;
            }
        }

        double acc_back = 0.0;
        size_t start_idx = 0;
        for (size_t i = closest; i > 0; --i) {
            const auto& a = path.poses[i];
            const auto& b = path.poses[i - 1];
            acc_back += std::hypot(a.pose.position.x - b.pose.position.x,
                                 a.pose.position.y - b.pose.position.y);
            start_idx = i - 1;
            if (acc_back >= backward) {
                break;
            }
        }

        commsgs::planning_msgs::Path out;
        out.header = path.header;
        out.poses.assign(path.poses.begin() + static_cast<std::ptrdiff_t>(start_idx),
                         path.poses.begin() +
                             static_cast<std::ptrdiff_t>(end_idx + 1));
        setOutput("output_path", out);
        setOutput(kBlackboardPathKey, out);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(TruncatePathLocalAction, "TruncatePathLocal")
