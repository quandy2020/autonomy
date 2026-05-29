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

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/control/controller_server.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/smoother_server.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"
#include "autonomy/transform/buffer.hpp"

namespace autolink {
class Node;
}

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/** Shared runtime state injected on the BT blackboard as "bt_context". */
struct BtContext {
    std::shared_ptr<planning::PlannerServer> planner;
    std::shared_ptr<planning::SmootherServer> smoother;
    std::shared_ptr<control::ControllerServer> controller;
    std::shared_ptr<transform::Buffer> tf_buffer;
    std::shared_ptr<autolink::Node> autolink_node;

    ::autonomy::tasks::proto::TaskOptions options;

    std::atomic<bool> cancel_requested{false};
    std::atomic<bool> pause_requested{false};
    std::atomic<bool> preempt_requested{false};
    std::function<void(const commsgs::planning_msgs::Path&)> on_path;

    int number_recoveries{0};

    /** Elapsed seconds for TimeExpired / local survival (reset per navigation). */
    std::chrono::steady_clock::time_point navigation_start{
        std::chrono::steady_clock::now()};

    std::function<bool()> CancelChecker() const {
        return [this]() { return cancel_requested.load(); };
    }

    bool IsPaused() const { return pause_requested.load(); }
};

constexpr char kBlackboardAutolinkNodeKey[] = "autolink_node";
constexpr char kBlackboardContextKey[] = "bt_context";
constexpr char kBlackboardGoalKey[] = "goal";
constexpr char kBlackboardGoalsKey[] = "goals";
constexpr char kBlackboardPathKey[] = "path";
constexpr char kBlackboardGlobalFrameKey[] = "global_frame";
constexpr char kBlackboardRobotBaseFrameKey[] = "robot_base_frame";
constexpr char kBlackboardInitialPoseReceivedKey[] = "initial_pose_received";
constexpr char kBlackboardBtLoopDurationKey[] = "bt_loop_duration";
constexpr char kBlackboardServerTimeoutKey[] = "server_timeout";
constexpr char kBlackboardNumberRecoveriesKey[] = "number_recoveries";
constexpr char kBlackboardLocalSurvivalTimeoutKey[] = "local_survival_timeout";
constexpr char kBlackboardGoalReachedTolKey[] = "goal_reached_tol";

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
