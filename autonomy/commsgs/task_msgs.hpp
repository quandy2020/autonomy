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

#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/proto/task_msgs.pb.h"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace task_msgs {

struct BehaviorTreeStatusChange {
    AUTONOMY_SMART_PTR_DEFINITIONS(BehaviorTreeStatusChange)

    builtin_interfaces::Time timestamp;
    std::string node_name;
    uint16_t uid{0};
    std::string previous_status;
    std::string current_status;
};

struct BehaviorTreeLog {
    AUTONOMY_SMART_PTR_DEFINITIONS(BehaviorTreeLog)

    builtin_interfaces::Time timestamp;
    std::vector<BehaviorTreeStatusChange> event_log;
};

struct SpeedLimit {
    AUTONOMY_SMART_PTR_DEFINITIONS(SpeedLimit)

    std_msgs::Header header;
    bool percentage{false};
    float speed_limit{0.0F};
};

struct CollisionMonitorState {
    AUTONOMY_SMART_PTR_DEFINITIONS(CollisionMonitorState)

    static constexpr uint8_t kDoNothing = 0;
    static constexpr uint8_t kStop = 1;
    static constexpr uint8_t kSlowdown = 2;
    static constexpr uint8_t kApproach = 3;
    static constexpr uint8_t kLimit = 4;

    uint8_t action_type{kDoNothing};
    std::string polygon_name;
};

struct CollisionDetectorState {
    AUTONOMY_SMART_PTR_DEFINITIONS(CollisionDetectorState)

    std::vector<std::string> polygons;
    std::vector<bool> detections;
};

struct WaypointStatus {
    AUTONOMY_SMART_PTR_DEFINITIONS(WaypointStatus)

    static constexpr uint8_t kPending = 0;
    static constexpr uint8_t kCompleted = 1;
    static constexpr uint8_t kSkipped = 2;
    static constexpr uint8_t kFailed = 3;

    uint8_t waypoint_status{kPending};
    uint32_t waypoint_index{0};
    geometry_msgs::PoseStamped waypoint_pose;
    uint16_t error_code{0};
    std::string error_msg;
};

struct IsPathValidRequest {
    planning_msgs::Path path;
    uint8_t max_cost{253};
    bool consider_unknown_as_obstacle{false};
};

struct IsPathValidResponse {
    bool is_valid{false};
    std::vector<int32_t> invalid_pose_indices;
};

struct SetInitialPoseRequest {
    geometry_msgs::PoseWithCovarianceStamped pose;
};

struct ManageLifecycleNodesRequest {
    static constexpr uint8_t kStartup = 0;
    static constexpr uint8_t kPause = 1;
    static constexpr uint8_t kResume = 2;
    static constexpr uint8_t kReset = 3;
    static constexpr uint8_t kShutdown = 4;
    static constexpr uint8_t kConfigure = 5;
    static constexpr uint8_t kCleanup = 6;

    uint8_t command{kStartup};
};

struct ManageLifecycleNodesResponse {
    bool success{false};
};

proto::task_msgs::SpeedLimit ToProto(const SpeedLimit& data);
SpeedLimit FromProto(const proto::task_msgs::SpeedLimit& proto);

}  // namespace task_msgs
}  // namespace commsgs
}  // namespace autonomy
