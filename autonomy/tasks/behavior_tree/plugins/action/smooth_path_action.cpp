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

#include <chrono>

#include "autonomy/common/logging.hpp"
#include "autonomy/planning/common/smoother_exceptions.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

namespace {

using ErrorCode = proto::SmoothPathErrorCode;

std::string ResolveSmootherId(const std::string& smoother_id,
                              const std::string& default_smoother_id) {
    if (smoother_id.empty()) {
        return default_smoother_id.empty() ? "simple_smoother"
                                           : default_smoother_id;
    }
    return smoother_id;
}

}  // namespace

SmoothPathAction::SmoothPathAction(const std::string& xml_tag_name,
                                   const BT::NodeConfiguration& conf)
    : BtStatefulActionNode(xml_tag_name, conf) {}

void SmoothPathAction::setFailure(int32_t code, const std::string& msg) {
    commsgs::planning_msgs::Path empty_path;
    setOutput("smoothed_path", empty_path);
    setOutput("smoothing_duration", 0.0);
    setOutput("was_completed", false);
    setOutput("error_code_id", code);
    setOutput("error_msg", msg);
}

BT::NodeStatus SmoothPathAction::onStart() {
    input_ready_ = false;

    auto ctx = taskContext();
    if (!ctx || !ctx->smoother) {
        setFailure(static_cast<int32_t>(ErrorCode::SMOOTH_PATH_ERROR_UNKNOWN),
                   "TaskContext or SmootherServer is not available.");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("unsmoothed_path", input_path_) ||
        input_path_.poses.size() < 2) {
        setFailure(static_cast<int32_t>(ErrorCode::SMOOTH_PATH_ERROR_INVALID_PATH),
                   "Input path must contain at least 2 poses.");
        return BT::NodeStatus::FAILURE;
    }

    getInput("max_smoothing_duration", max_smoothing_duration_);
    getInput("check_for_collisions", check_for_collisions_);

    std::string smoother_input;
    getInput("smoother_id", smoother_input);
    smoother_id_ =
        ResolveSmootherId(smoother_input, ctx->selected_smoother_id);

    input_ready_ = true;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SmoothPathAction::onRunning() {
    if (!input_ready_) {
        return BT::NodeStatus::FAILURE;
    }

    auto ctx = taskContext();
    if (!ctx || !ctx->smoother) {
        setFailure(static_cast<int32_t>(ErrorCode::SMOOTH_PATH_ERROR_UNKNOWN),
                   "SmootherServer is not available.");
        return BT::NodeStatus::FAILURE;
    }

    if (ctx->IsCancelRequested()) {
        setFailure(static_cast<int32_t>(ErrorCode::SMOOTH_PATH_ERROR_NONE), "");
        return BT::NodeStatus::FAILURE;
    }

    const auto max_time = std::chrono::milliseconds(static_cast<int>(
        max_smoothing_duration_ * 1000.0));

    try {
        const planning::SmoothPathResult result = ctx->smoother->SmoothPath(
            input_path_, smoother_id_, max_time, check_for_collisions_,
            ctx->CancelChecker());
        setOutput("smoothed_path", result.path);
        setOutput("smoothing_duration", result.smoothing_duration_sec);
        setOutput("was_completed", result.was_completed);
        setOutput("error_code_id",
                  static_cast<int32_t>(ErrorCode::SMOOTH_PATH_ERROR_NONE));
        setOutput("error_msg", std::string(""));
        return BT::NodeStatus::SUCCESS;
    } catch (const planning::common::InvalidSmoother& ex) {
        AERROR << "SmoothPath: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::SMOOTH_PATH_ERROR_INVALID_SMOOTHER),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::InvalidPath& ex) {
        AERROR << "SmoothPath: " << ex.what();
        setFailure(static_cast<int32_t>(ErrorCode::SMOOTH_PATH_ERROR_INVALID_PATH),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::SmootherTimedOut& ex) {
        AERROR << "SmoothPath: " << ex.what();
        setFailure(static_cast<int32_t>(ErrorCode::SMOOTH_PATH_ERROR_TIMEOUT),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::SmoothedPathInCollision& ex) {
        AERROR << "SmoothPath: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::SMOOTH_PATH_ERROR_SMOOTHED_PATH_IN_COLLISION),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::FailedToSmoothPath& ex) {
        AERROR << "SmoothPath: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::SMOOTH_PATH_ERROR_FAILED_TO_SMOOTH_PATH),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const std::exception& ex) {
        AERROR << "SmoothPath: " << ex.what();
        setFailure(static_cast<int32_t>(ErrorCode::SMOOTH_PATH_ERROR_UNKNOWN),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    }
}

void SmoothPathAction::onHalted() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("smoothed_path", empty_path);
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<
            autonomy::tasks::behavior_tree::plugins::action::SmoothPathAction>(
            name, config);
    };

    factory.registerBuilder<
        autonomy::tasks::behavior_tree::plugins::action::SmoothPathAction>(
        "SmoothPath", builder);
}
