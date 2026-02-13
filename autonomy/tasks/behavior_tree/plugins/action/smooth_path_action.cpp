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

#include "autonomy/tasks/behavior_tree/plugins/action/smooth_path_action.hpp"

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

SmoothPathAction::SmoothPathAction(const std::string& xml_tag_name, const std::string& action_name,
                                   const BT::NodeConfiguration& conf)
    : BtActionNode<proto::SmoothPathAction>(xml_tag_name, action_name, conf) {}

void SmoothPathAction::on_tick() {
    commsgs::planning_msgs::Path path;
    getInput("unsmoothed_path", path);
    auto* proto_path = goal_.mutable_path();
    *proto_path->mutable_header() = commsgs::std_msgs::ToProto(path.header);
    proto_path->clear_poses();
    for (const auto& pose : path.poses) {
        auto* proto_pose = proto_path->add_poses();
        *proto_pose = commsgs::geometry_msgs::ToProto(pose);
    }

    std::string smoother_id;
    getInput("smoother_id", smoother_id);
    goal_.set_smoother_id(smoother_id);

    double max_smoothing_duration;
    getInput("max_smoothing_duration", max_smoothing_duration);
    auto duration = commsgs::builtin_interfaces::Duration::FromSeconds(max_smoothing_duration);
    int64_t ns = duration.Nanoseconds();
    int32_t sec = static_cast<int32_t>(ns / 1'000'000'000LL);
    uint32_t nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
    goal_.mutable_max_smoothing_duration()->mutable_stamp()->set_sec(sec);
    goal_.mutable_max_smoothing_duration()->mutable_stamp()->set_nanosec(nanosec);

    bool check_for_collisions;
    getInput("check_for_collisions", check_for_collisions);
    goal_.set_check_for_collisions(check_for_collisions);
}

BT::NodeStatus SmoothPathAction::on_success() {
    if (result_.result && result_.result->has_path()) {
        commsgs::planning_msgs::Path path;
        const auto& proto_path = result_.result->path();
        path.header = commsgs::std_msgs::FromProto(proto_path.header());
        for (const auto& proto_pose : proto_path.poses()) {
            path.poses.push_back(commsgs::geometry_msgs::FromProto(proto_pose));
        }
        setOutput("smoothed_path", path);
    } else {
        commsgs::planning_msgs::Path empty_path;
        setOutput("smoothed_path", empty_path);
    }

    if (result_.result && result_.result->has_smoothing_duration()) {
        const auto& proto_duration = result_.result->smoothing_duration();
        double seconds = proto_duration.stamp().sec() + proto_duration.stamp().nanosec() / 1000000000.0;
        setOutput("smoothing_duration", seconds);
    } else {
        setOutput("smoothing_duration", 0.0);
    }

    bool was_completed = result_.result ? result_.result->was_completed() : false;
    setOutput("was_completed", was_completed);

    // Set empty error code, action was successful
    setOutput("error_code_id", static_cast<int32_t>(proto::SmoothPathErrorCode::SMOOTH_PATH_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SmoothPathAction::on_aborted() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("smoothed_path", empty_path);
    if (result_.result) {
        setOutput("error_code_id", static_cast<int32_t>(result_.result->error_code()));
        setOutput("error_msg", result_.result->error_msg());
    } else {
        setOutput("error_code_id", static_cast<int32_t>(proto::SmoothPathErrorCode::SMOOTH_PATH_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Unknown error"));
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SmoothPathAction::on_cancelled() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("smoothed_path", empty_path);
    // Set empty error code, action was cancelled
    setOutput("error_code_id", static_cast<int32_t>(proto::SmoothPathErrorCode::SMOOTH_PATH_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

void SmoothPathAction::on_timeout() {
    setOutput("error_code_id", static_cast<int32_t>(proto::SmoothPathErrorCode::SMOOTH_PATH_ERROR_TIMEOUT));
    setOutput("error_msg", std::string("Behavior Tree action client timed out waiting."));
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<autonomy::tasks::behavior_tree::plugins::action::SmoothPathAction>(name, "smooth_path",
                                                                                                   config);
    };

    factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::SmoothPathAction>("SmoothPath", builder);
}