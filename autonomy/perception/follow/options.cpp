/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/follow/options.hpp"

#include <cmath>
#include <string>

namespace autonomy {
namespace perception {
namespace follow {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Follow: " + message;
    }
}

bool AbsoluteTopic(const std::string& topic) {
    return !topic.empty() && topic.front() == '/';
}

}  // namespace

bool ValidateFollowOptions(const proto::FollowOptions& options,
                           std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    const char* topics[] = {
        options.tracks_topic().c_str(), options.depth_topic().c_str(),
        options.camera_info_topic().c_str(), options.odom_topic().c_str(),
        options.select_topic().c_str(), options.target_topic().c_str(),
        options.path_topic().c_str(), options.grid_topic().c_str()};
    for (const char* topic : topics) {
        if (!AbsoluteTopic(topic)) {
            SetError(error, "all topics must be absolute paths.");
            return false;
        }
    }
    if (options.camera_frame().empty() || options.base_frame().empty() ||
        options.map_frame().empty()) {
        SetError(error, "camera_frame, base_frame, and map_frame are required.");
        return false;
    }
    if (!(options.follow_distance_m() > 0.0F) ||
        !(options.grid_length_m() > 0.0F) ||
        !(options.grid_resolution_m() > 0.0F) ||
        !(options.path_step_m() > 0.0F) || options.path_max_poses() == 0 ||
        options.min_depth_samples() == 0) {
        SetError(error, "follow geometry parameters must be positive.");
        return false;
    }
    if (!(options.min_depth_m() > 0.0F) ||
        !(options.max_depth_m() > options.min_depth_m())) {
        SetError(error, "depth range is invalid.");
        return false;
    }
    return true;
}

}  // namespace follow
}  // namespace perception
}  // namespace autonomy
