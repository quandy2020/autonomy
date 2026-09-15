/*
 * Copyright 2026 The Openbot Authors
 *
 * Drop waypoints that sit in lethal / inscribed costmap cells so through-poses
 * planning can continue over the remaining free goals in order.
 */

#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

class RemoveInCollisionGoalsAction : public BtSyncAction
{
public:
    RemoveInCollisionGoalsAction(const std::string& name,
                                 const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<
                std::vector<automsgs::msgs::geometry_msgs::PoseStamped>>(
                "input_goals"),
            BT::OutputPort<
                std::vector<automsgs::msgs::geometry_msgs::PoseStamped>>(
                "output_goals"),
            BT::InputPort<int>("cost_threshold", 253,
                               "reject goals with cost >= threshold"),
            BT::InputPort<bool>("consider_unknown_as_obstacle", false, ""),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        std::vector<automsgs::msgs::geometry_msgs::PoseStamped> goals;
        if (!getInput("input_goals", goals)) {
            return BT::NodeStatus::FAILURE;
        }

        int cost_threshold = 253;
        bool consider_unknown = false;
        getInput("cost_threshold", cost_threshold);
        getInput("consider_unknown_as_obstacle", consider_unknown);

        auto client = ResolveClient(*this);
        std::vector<automsgs::msgs::geometry_msgs::PoseStamped> kept;
        kept.reserve(goals.size());
        size_t removed = 0;

        for (const auto& goal : goals) {
            automsgs::msgs::nav_msgs::Path probe;
            if (goal.has_header()) {
                *probe.mutable_header() = goal.header();
            }
            *probe.add_poses() = goal;
            if (!client->IsPathValid(probe,
                                     static_cast<uint8_t>(cost_threshold),
                                     consider_unknown)) {
                ++removed;
                AINFO_EVERY(5)
                    << "RemoveInCollisionGoals: drop ("
                    << goal.pose().position().x() << ", "
                    << goal.pose().position().y() << ")";
                continue;
            }
            kept.push_back(goal);
        }

        if (removed > 0) {
            AINFO << "RemoveInCollisionGoals: removed " << removed
                  << " in-collision waypoint(s); remaining=" << kept.size();
        }

        setOutput("output_goals", kept);
        if (config().blackboard) {
            config().blackboard->set("goals", kept);
            config().blackboard->set("number_of_goals",
                                     static_cast<int>(kept.size()));
            if (!kept.empty()) {
                config().blackboard->set("goal", kept.back());
            }
        }
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::RemoveInCollisionGoalsAction>(
        "RemoveInCollisionGoals");
}
