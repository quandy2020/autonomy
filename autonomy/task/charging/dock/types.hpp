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
 * @file types.hpp
 * @brief Dock instance and plugin maps.
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autonomy/task/charging/dock/charging_dock.hpp"

namespace autonomy {
namespace task {
namespace charging {

/**
 * @brief One named charger: pose, plugin type, and bound ChargingDock.
 */
struct Dock {
    ::automsgs::msgs::geometry_msgs::PoseStamped GetStagingPose() const {
        if (!plugin) {
            ::automsgs::msgs::geometry_msgs::PoseStamped staged;
            staged.mutable_header()->set_frame_id(frame);
            *staged.mutable_pose() = pose;
            return staged;
        }
        return plugin->GetStagingPose(pose, frame);
    }

    ::automsgs::msgs::geometry_msgs::Pose pose;
    std::string frame{"map"};
    std::string type;
    std::string id;
    ChargingDock::Ptr plugin;
};

using DockPluginMap = std::unordered_map<std::string, ChargingDock::Ptr>;
using DockMap = std::unordered_map<std::string, Dock>;

}  // namespace charging
}  // namespace task
}  // namespace autonomy
