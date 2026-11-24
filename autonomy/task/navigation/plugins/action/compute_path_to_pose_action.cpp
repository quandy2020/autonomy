/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file compute_path_to_pose_action.cpp
 * @brief Compute path to pose action.
 */

#include <cmath>
#include <cstdint>
#include "autonomy/task/navigation/plugins/async_path_action_node.hpp"

namespace autonomy::task::plugins::navigation {
namespace actions = automsgs::actions;
class ComputePathAction : public AsyncPathActionNode<actions::ComputePathToPoseAction>
{
public:
    using AsyncPathActionNode::AsyncPathActionNode;
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<automsgs::msgs::geometry_msgs::PoseStamped>("goal"),
            BT::BidirectionalPort<automsgs::msgs::nav_msgs::Path>("path"),
            BT::InputPort<std::string>("planner_id"),
            BT::InputPort<double>("max_path_reuse_age", 2.0, "s"),
            BT::OutputPort<uint64_t>("path_revision"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }
protected:
    Client& GetClient(NavClient& client) override { return client.compute_path_client(); }
    bool BuildGoal(Goal& goal, Pose& endpoint) override
    {
        if (!getInput("goal", endpoint)) return false;
        *goal.mutable_goal() = endpoint;
        std::string planner;
        getInput("planner_id", planner);
        goal.set_planner_id(planner);
        goal.set_use_start(false);
        return true;
    }
    void OutputPath(const Path& path, bool) override
    {
        Path previous;
        const bool changed = output_epoch_ != GoalEpoch() || !getInput("path", previous) ||
            (!previous.poses().empty() || !path.poses().empty()) &&
            !::autonomy::task::navigation::EquivalentPathSuffix(previous, path);
        if (changed) {
            if (config().blackboard) config().blackboard->get("path_revision", path_revision_);
            ++path_revision_;
        }
        setOutput("path", path);
        setOutput("path_revision", path_revision_);
        if (config().blackboard) config().blackboard->set("path_revision", path_revision_);
        output_epoch_ = GoalEpoch();
    }
    uint64_t path_revision_{0};
    int output_epoch_{-1};
};
}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::navigation::ComputePathAction>("PlanPath");
    factory.registerNodeType<autonomy::task::plugins::navigation::ComputePathAction>("ComputePathToPose");
}
