/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/tasks/behavior_tree/plugins/decorator/goal_updater_node.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace decorator {

GoalUpdater::GoalUpdater(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::DecoratorNode(name, conf), goal_updater_topic_("goal_update"), goals_updater_topic_("goals_update") {
    initialize();
}

void GoalUpdater::initialize() {
    createROSInterfaces();
}

void GoalUpdater::createROSInterfaces() {
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    std::string goal_updater_topic_new;
    std::string goals_updater_topic_new;
    goal_updater_topic_ = goal_updater_topic_new;
    goals_updater_topic_ = goals_updater_topic_new;
    goal_sub_ = node_->CreateReader<commsgs::geometry_msgs::PoseStamped>(
        goal_updater_topic_, std::bind(&GoalUpdater::callback_updated_goal, this, std::placeholders::_1));
    goals_sub_ = node_->CreateReader<commsgs::planning_msgs::Goals>(
        goals_updater_topic_, std::bind(&GoalUpdater::callback_updated_goals, this, std::placeholders::_1));
}

inline BT::NodeStatus GoalUpdater::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    commsgs::geometry_msgs::PoseStamped goal;
    commsgs::planning_msgs::Goals goals;

    getInput("input_goal", goal);
    getInput("input_goals", goals);

    if (last_goal_received_set_) {
        // Check if timestamp is zero (no timestamp)
        bool has_timestamp =
            !(last_goal_received_.header.stamp.sec == 0 && last_goal_received_.header.stamp.nanosec == 0);
        if (!has_timestamp) {
            // if the goal doesn't have a timestamp, we reject it
            AWARN << "The received goal has no timestamp. Ignoring.";
            setOutput("output_goal", goal);
        } else {
            // Compare timestamps manually
            int64_t last_time_ns = static_cast<int64_t>(last_goal_received_.header.stamp.sec) * 1000000000LL +
                                   static_cast<int64_t>(last_goal_received_.header.stamp.nanosec);
            int64_t goal_time_ns = static_cast<int64_t>(goal.header.stamp.sec) * 1000000000LL +
                                   static_cast<int64_t>(goal.header.stamp.nanosec);
            if (last_time_ns >= goal_time_ns) {
                setOutput("output_goal", last_goal_received_);
            } else {
                AINFO << "The timestamp of the received goal is older than the "
                         "current goal. Ignoring the received goal.";
                setOutput("output_goal", goal);
            }
        }
    } else {
        setOutput("output_goal", goal);
    }

    if (last_goals_received_set_) {
        if (last_goals_received_.goals.empty()) {
            setOutput("output_goals", goals);
        } else {
            // Check if timestamp is zero
            bool has_timestamp =
                !(last_goals_received_.header.stamp.sec == 0 && last_goals_received_.header.stamp.nanosec == 0);
            if (!has_timestamp) {
                AWARN << "The received goals array has no timestamp. Ignoring.";
                setOutput("output_goals", goals);
            } else {
                // Compare timestamps manually
                int64_t last_time_ns = static_cast<int64_t>(last_goals_received_.header.stamp.sec) * 1000000000LL +
                                       static_cast<int64_t>(last_goals_received_.header.stamp.nanosec);
                int64_t goals_time_ns = static_cast<int64_t>(goals.header.stamp.sec) * 1000000000LL +
                                        static_cast<int64_t>(goals.header.stamp.nanosec);
                if (last_time_ns >= goals_time_ns) {
                    setOutput("output_goals", last_goals_received_);
                } else {
                    AINFO << "The timestamp of the received goals is older than "
                             "the current goals. Ignoring the received goals.";
                    setOutput("output_goals", goals);
                }
            }
        }
    } else {
        setOutput("output_goals", goals);
    }

    return child_node_->executeTick();
}

void GoalUpdater::callback_updated_goal(const commsgs::geometry_msgs::PoseStamped::SharedPtr msg) {
    last_goal_received_ = *msg;
    last_goal_received_set_ = true;
}

void GoalUpdater::callback_updated_goals(const commsgs::planning_msgs::Goals::SharedPtr msg) {
    last_goals_received_ = *msg;
    last_goals_received_set_ = true;
}

}  // namespace decorator
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::decorator::GoalUpdater>("GoalUpdater");
}
