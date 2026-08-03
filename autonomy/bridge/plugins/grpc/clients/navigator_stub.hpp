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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/node_client.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/macros.hpp"
#include <automsgs/actions/nav_actions.pb.h>

namespace autonomy {
namespace bridge {
namespace plugins {
namespace grpc {
namespace clients {

/**
 * @brief Forwards bridge NavigationCommand RPCs to navigator action servers.
 *
 * Uses NodeClient to call navigate_to_pose / navigate_through_poses
 * (see bridge/constants.hpp). Stream callbacks emit NavigationCommandResponse
 * frames for gRPC server-streaming handlers.
 */
class NavigatorStub
{
public:
    using StreamCallback =
        std::function<void(const proto::NavigationCommandResponse& response)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(NavigatorStub)

    explicit NavigatorStub(std::shared_ptr<autolink::Node> node);

    bool ServersReady() const;
    bool WaitForServers(
        std::chrono::milliseconds timeout = std::chrono::seconds(10));
    bool IsNavigating() const;

    /**
     * @brief Dispatch a navigation command to the appropriate action server.
     *
     * @return false when the command is rejected immediately (validation,
     *         server unavailable, or another goal is active).
     */
    bool HandleCommand(const proto::NavigationCommandRequest& request,
                       StreamCallback stream_callback);

private:
    enum class ActiveMode { NONE, TO_POSE, THROUGH_POSES };

    using NavigateToPoseClient =
        NodeClient<automsgs::actions::NavigateToPoseAction>;
    using NavigateThroughPosesClient =
        NodeClient<automsgs::actions::NavigateThroughPosesAction>;
    using ToPoseGoalHandle = NavigateToPoseClient::GoalHandle;
    using ThroughPosesGoalHandle = NavigateThroughPosesClient::GoalHandle;

    bool StartToPose(const proto::NavigationCommandRequest& request,
                     StreamCallback stream_callback);
    bool StartThroughPoses(const proto::NavigationCommandRequest& request,
                           StreamCallback stream_callback);
    bool CancelActive(const proto::NavigationCommandRequest& request,
                      StreamCallback stream_callback);

    void ClearActiveGoalLocked();
    proto::NavigationCommandResponse MakeResponse(
        const proto::NavigationCommandRequest& request,
        proto::NavigationStatus status, bool success, bool final,
        const std::string& message = "") const;

    NavigateToPoseClient::SharedPtr navigate_to_pose_client_;
    NavigateThroughPosesClient::SharedPtr navigate_through_poses_client_;

    mutable std::mutex mutex_;
    ActiveMode active_mode_{ActiveMode::NONE};
    std::shared_ptr<ToPoseGoalHandle> to_pose_handle_;
    std::shared_ptr<ThroughPosesGoalHandle> through_poses_handle_;
    int total_waypoints_{0};
};

}  // namespace clients
}  // namespace grpc
}  // namespace plugins
}  // namespace bridge
}  // namespace autonomy
