/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/decorator_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class PathLongerOnApproach : public BT::DecoratorNode
{
public:
    PathLongerOnApproach(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::DecoratorNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::planning_msgs::Path>("path"),
            BT::InputPort<double>("prox_len", 3.0, "Proximity length"),
            BT::InputPort<double>("length_factor", 2.0, "Length factor"),
        };
    }

    BT::NodeStatus tick() override {
        commsgs::planning_msgs::Path new_path;
        GetInputOrBlackboard(*this, config(), "path", kBlackboardPathKey, new_path);
        double prox_len = 3.0;
        double length_factor = 2.0;
        getInput("prox_len", prox_len);
        getInput("length_factor", length_factor);

        if (!first_time_ && !old_path_.poses.empty() && !new_path.poses.empty() &&
            !PoseStampedEqual(old_path_.poses.back(), new_path.poses.back())) {
            first_time_ = true;
        }
        setStatus(BT::NodeStatus::RUNNING);

        const bool updated =
            !old_path_.poses.empty() && !new_path.poses.empty() &&
            new_path.poses.size() != old_path_.poses.size() &&
            PoseStampedEqual(old_path_.poses.back(), new_path.poses.back());
        const bool near_goal =
            !old_path_.poses.empty() &&
            map::costmap_2d::utils::calculate_path_length(old_path_) <
                prox_len;
        const bool longer =
            !old_path_.poses.empty() &&
            map::costmap_2d::utils::calculate_path_length(new_path) >
                length_factor *
                    map::costmap_2d::utils::calculate_path_length(old_path_);

        if (updated && near_goal && longer && !first_time_) {
            const BT::NodeStatus child_state = child_node_->executeTick();
            if (child_state == BT::NodeStatus::SUCCESS ||
                child_state == BT::NodeStatus::FAILURE) {
                old_path_ = new_path;
                resetChild();
            }
            return child_state;
        }
        old_path_ = new_path;
        first_time_ = false;
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool first_time_{true};
    commsgs::planning_msgs::Path old_path_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(PathLongerOnApproach, "PathLongerOnApproach")
