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

#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/proto/planning_msgs.pb.h"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace planning_msgs {

struct Path {
    AUTONOMY_SMART_PTR_DEFINITIONS(Path)

    std_msgs::Header header;
    std::vector<geometry_msgs::PoseStamped> poses;
};

struct Odometry {
    AUTONOMY_SMART_PTR_DEFINITIONS(Odometry)

    std_msgs::Header header;
    std::string child_frame_id;
    geometry_msgs::PoseWithCovariance pose;
    geometry_msgs::TwistWithCovariance twist;
};

struct Goals {
    AUTONOMY_SMART_PTR_DEFINITIONS(Goals)

    std_msgs::Header header;
    std::vector<geometry_msgs::PoseStamped> goals;
};

proto::planning_msgs::Path ToProto(const Path& data);
Path FromProto(const proto::planning_msgs::Path& proto);

}  // namespace planning_msgs
}  // namespace commsgs
}  // namespace autonomy
