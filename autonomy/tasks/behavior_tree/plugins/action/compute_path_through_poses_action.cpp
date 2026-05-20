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

#include "autonomy/tasks/behavior_tree/plugins/action/compute_path_through_poses_action.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/tasks/utils/planner_id_utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

namespace {

using ErrorCode = proto::ComputePathThroughPosesErrorCode;

}  // namespace

ComputePathThroughPosesAction::ComputePathThroughPosesAction(
    const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BtStatefulActionNode(xml_tag_name, conf) {}

void ComputePathThroughPosesAction::setFailure(int32_t code,
                                               const std::string& msg) {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    setOutput("error_code_id", code);
    setOutput("error_msg", msg);
}

BT::NodeStatus ComputePathThroughPosesAction::onStart() {
    poses_ready_ = false;
    goal_poses_.clear();

    auto ctx = taskContext();
    if (!ctx || !ctx->planner) {
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_UNKNOWN),
                   "TaskContext or PlannerServer is not available.");
        return BT::NodeStatus::FAILURE;
    }

    commsgs::planning_msgs::Goals goals;
    if (!getInput("goals", goals) || goals.goals.empty()) {
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_NO_VIAPOINTS_GIVEN),
                   "Missing or empty goals port.");
        return BT::NodeStatus::FAILURE;
    }

    goal_poses_ = goals.goals;

    bool use_start = false;
    getInput("use_start", use_start);
    if (use_start) {
        commsgs::geometry_msgs::PoseStamped start;
        if (!getInput("start", start)) {
            setFailure(static_cast<int32_t>(
                           ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_UNKNOWN),
                       "Missing required port: start");
            return BT::NodeStatus::FAILURE;
        }
        start_pose_ = start;
    } else {
        if (!utils::getGlobalRobotPose(start_pose_, ctx->tf, ctx->odom_smoother,
                                       ctx->global_frame, ctx->robot_base_frame,
                                       static_cast<float>(
                                           ctx->transform_tolerance))) {
            setFailure(static_cast<int32_t>(
                           ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_TF_ERROR),
                       "Robot pose is not available.");
            return BT::NodeStatus::FAILURE;
        }
    }

    std::string planner_input;
    getInput("planner_id", planner_input);
    planner_id_ =
        utils::ResolvePlannerId(planner_input, ctx->selected_planner_id);
    poses_ready_ = true;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputePathThroughPosesAction::onRunning() {
    if (!poses_ready_) {
        return BT::NodeStatus::FAILURE;
    }

    auto ctx = taskContext();
    if (!ctx || !ctx->planner) {
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_UNKNOWN),
                   "PlannerServer is not available.");
        return BT::NodeStatus::FAILURE;
    }

    if (ctx->IsCancelRequested()) {
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_NONE),
                   "");
        return BT::NodeStatus::FAILURE;
    }

    try {
        commsgs::planning_msgs::Path path =
            ctx->planner->ComputePathThroughPoses(
                start_pose_, goal_poses_, planner_id_, ctx->CancelChecker());
        setOutput("path", path);
        setOutput("error_code_id",
                  static_cast<int32_t>(
                      ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_NONE));
        setOutput("error_msg", std::string(""));
        return BT::NodeStatus::SUCCESS;
    } catch (const planning::common::PlannerCancelled&) {
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_NONE),
                   "");
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::InvalidPlanner& ex) {
        AERROR << "ComputePathThroughPoses: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_INVALID_PLANNER),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::StartOccupied& ex) {
        AERROR << "ComputePathThroughPoses: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_START_OCCUPIED),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::GoalOccupied& ex) {
        AERROR << "ComputePathThroughPoses: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_GOAL_OCCUPIED),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::NoValidPathCouldBeFound& ex) {
        AERROR << "ComputePathThroughPoses: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_NO_VALID_PATH),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::PlannerTimedOut& ex) {
        AERROR << "ComputePathThroughPoses: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_TIMEOUT),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::StartOutsideMapBounds& ex) {
        AERROR << "ComputePathThroughPoses: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_START_OUTSIDE_MAP),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::GoalOutsideMapBounds& ex) {
        AERROR << "ComputePathThroughPoses: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_GOAL_OUTSIDE_MAP),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::PlannerTFError& ex) {
        AERROR << "ComputePathThroughPoses: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_TF_ERROR),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const planning::common::NoViapointsGiven& ex) {
        AERROR << "ComputePathThroughPoses: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_NO_VIAPOINTS_GIVEN),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    } catch (const std::exception& ex) {
        AERROR << "ComputePathThroughPoses: " << ex.what();
        setFailure(static_cast<int32_t>(
                       ErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_UNKNOWN),
                   ex.what());
        return BT::NodeStatus::FAILURE;
    }
}

void ComputePathThroughPosesAction::onHalted() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
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
        return std::make_unique<autonomy::tasks::behavior_tree::plugins::
                                    action::ComputePathThroughPosesAction>(
            name, config);
    };

    factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::
                                ComputePathThroughPosesAction>(
        "ComputePathThroughPoses", builder);
}
