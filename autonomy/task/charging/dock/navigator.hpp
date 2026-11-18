/*
 * Copyright 2026 The Openbot Authors
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
 * @file navigator.hpp
 * @brief Tick-based staging navigation (port of opennav_docking::Navigator).
 */

#ifndef AUTONOMY_TASK_CHARGING_DOCK_NAVIGATOR_HPP_
#define AUTONOMY_TASK_CHARGING_DOCK_NAVIGATOR_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autonomy/task/navigation/navigation_client.hpp"

namespace autonomy {
namespace task {
namespace charging {

/**
 * @class autonomy::task::charging::DockNavigator
 * @brief Plan + smooth + FollowPath to the staging pose, with one retry.
 *
 * ChargingTask cannot call /navigate_to_pose (same process). This matches
 * OpenNav Navigator::goToPose by using the navigation RPC stack and a single
 * recursive retry on remaining timeout.
 */
class DockNavigator {
public:
    enum class Status { Idle, Running, Succeeded, Failed, Canceled };

    /**
     * @brief Bind the navigation RPC client and planner/controller/smoother ids.
     * @param[in] client Shared NavigationClient; required before Begin().
     * @param[in] planner_id Planner id; empty keeps the default.
     * @param[in] controller_id FollowPath controller id; empty keeps default.
     * @param[in] smoother_id SmoothPath id; empty keeps default.
     */
    void Configure(navigation::NavigationClient::Ptr client,
                   std::string planner_id, std::string controller_id,
                   std::string smoother_id = {});

    /**
     * @brief Start navigation to @p pose.
     * @param[in] pose Staging pose.
     * @param[in] timeout Maximum staging duration.
     * @return true when FollowPath was accepted.
     * @throws FailedToStage when planning or follow cannot start.
     */
    bool Begin(const ::automsgs::msgs::geometry_msgs::PoseStamped& pose,
               std::chrono::milliseconds timeout);

    /**
     * @brief Advance FollowPath; retries the full plan+follow once on failure.
     * @param[in] canceled true to abort immediately.
     * @return Current status.
     * @throws FailedToStage on timeout or exhausted retry.
     */
    Status Tick(bool canceled);

    /**
     * @brief Cancel an in-flight FollowPath.
     */
    void Cancel();

private:
    /**
     * @brief ComputePathToPose, SmoothPath, then FollowPath::Begin.
     * @return true when follow was accepted.
     */
    bool LaunchFollow();

    navigation::NavigationClient::Ptr nav_;
    std::string planner_id_{"navfn_planner"};
    std::string controller_id_{"FollowPath"};
    std::string smoother_id_{"simple_smoother"};
    ::automsgs::msgs::geometry_msgs::PoseStamped pose_;
    Status status_{Status::Idle};
    std::chrono::steady_clock::time_point deadline_{};
    bool sent_{false};
    bool recursed_{false};
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_CHARGING_DOCK_NAVIGATOR_HPP_
