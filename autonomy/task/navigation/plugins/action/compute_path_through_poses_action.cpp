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
 * @file compute_path_through_poses_action.cpp
 * @brief Compute path through poses action.
 */

#include <cmath>
#include <cstdint>
#include "autonomy/task/navigation/plugins/async_path_action_node.hpp"

namespace autonomy::task::plugins::navigation {
namespace actions = automsgs::actions;
class ComputePathThroughPosesAction : public AsyncPathActionNode<actions::ComputePathThroughPosesAction>
{
public:
    using AsyncPathActionNode::AsyncPathActionNode;
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<automsgs::msgs::geometry_msgs::PoseStamped>>("goals"),
            BT::BidirectionalPort<automsgs::msgs::nav_msgs::Path>("path"),
            BT::InputPort<std::string>("planner_id"),
            BT::InputPort<double>("max_path_reuse_age", 2.0, "s"),
            BT::OutputPort<uint64_t>("path_revision"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }
protected:
    Client& GetClient(NavClient& client) override { return client.compute_through_poses_client(); }
    bool BuildGoal(Goal& goal, Pose& endpoint) override
    {
        std::vector<Pose> goals;
        empty_goals_ = getInput("goals", goals) && goals.empty();
        if (goals.empty()) return false;
        endpoint = goals.back();
        last_endpoint_ = endpoint;
        has_endpoint_ = true;
        endpoint_epoch_ = GoalEpoch();
        for (const auto& pose : goals) *goal.mutable_goals()->add_goals() = pose;
        *goal.mutable_goals()->mutable_header() = endpoint.header();
        std::string planner;
        getInput("planner_id", planner);
        goal.set_planner_id(planner);
        goal.set_use_start(false);
        return true;
    }
    bool EmptyGoalIsComplete() const override
    {
        // Waypoint filtering can empty the list on XY distance alone. Completion
        // still requires a fresh pose, final yaw and the same task epoch.
        return empty_goals_ && has_endpoint_ && endpoint_epoch_ == GoalEpoch() &&
            ResolveClient(*this)->IsGoalReached(last_endpoint_, 0.10, 0.10);
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
    Pose last_endpoint_;
    int endpoint_epoch_{-1};
    bool empty_goals_{false}, has_endpoint_{false};
};
}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::navigation::ComputePathThroughPosesAction>("PlanPoses");
    factory.registerNodeType<autonomy::task::plugins::navigation::ComputePathThroughPosesAction>("ComputePathThroughPoses");
}
