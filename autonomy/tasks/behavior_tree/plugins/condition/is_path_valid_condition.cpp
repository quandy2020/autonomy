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

#include "autonomy/tasks/behavior_tree/plugins/condition/is_path_valid_condition.hpp"

#include <future>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

IsPathValidCondition::IsPathValidCondition(const std::string& condition_name, const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf), max_cost_(253), consider_unknown_as_obstacle_(false) {
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    client_ = node_->CreateClient<proto::IsPathValid::Request, proto::IsPathValid::Response>("is_path_valid");

    server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
}

void IsPathValidCondition::initialize() {
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    getInput<unsigned int>("max_cost", max_cost_);
    getInput<bool>("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);
}

BT::NodeStatus IsPathValidCondition::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    commsgs::planning_msgs::Path path;
    getInput("path", path);

    auto request = std::make_shared<proto::IsPathValid::Request>();

    // Convert path to proto format manually
    auto* path_proto = request->mutable_path();
    *path_proto->mutable_header() = commsgs::std_msgs::ToProto(path.header);
    for (const auto& pose : path.poses) {
        *path_proto->add_poses() = commsgs::geometry_msgs::ToProto(pose);
    }
    request->set_max_cost(max_cost_);
    request->set_consider_unknown_as_obstacle(consider_unknown_as_obstacle_);

    auto future_result = client_->AsyncSendRequest(request);
    auto status = future_result.wait_for(server_timeout_);

    if (status == std::future_status::ready) {
        auto response = future_result.get();
        if (response && response->is_valid()) {
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::FAILURE;
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::condition::IsPathValidCondition>("IsPathValid");
}
