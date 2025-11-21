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

#include <vector>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace planning_msgs {

// An array of poses that represents a Path for a robot to follow.
struct Path 
{
    // Define Path::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(Path)

    // Indicates the frame_id of the path.
    std_msgs::Header header;

    // Array of poses to follow.
    std::vector<geometry_msgs::PoseStamped> poses;
};

struct Odometry
{
    // Define Odometry::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(Odometry)

    // This represents an estimate of a position and velocity in free space.
    // The pose in this message should be specified in the coordinate frame given by header.frame_id
    // The twist in this message should be specified in the coordinate frame given by the child_frame_id

    // Includes the frame id of the pose parent.
    std_msgs::Header header;

    // Frame id the pose points to. The twist is in this coordinate frame.
    std::string child_frame_id;

    // Estimated pose that is typically relative to a fixed world frame.
    geometry_msgs::PoseWithCovariance pose;

    // Estimated linear and angular velocity relative to child_frame_id.
    geometry_msgs::TwistWithCovariance twist;
};

struct CostmapFilterInfo
{
    // Define CostmapFilterInfo::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(CostmapFilterInfo)

    std_msgs::Header header;

    // Type of plugin used (keepout filter, speed limit in m/s, speed limit in percent, etc...)
    // 0: keepout/lanes filter
    // 1: speed limit filter in % of maximum speed
    // 2: speed limit filter in absolute values (m/s)
    uint32 type;

    // Name of filter mask topic
    std::string filter_mask_topic;

    // Multiplier base offset and multiplier coefficient for conversion of OccGrid.
    // Used to convert OccupancyGrid data values to filter space values.
    // data -> into some other number space:
    // space = data * multiplier + base
    float base;
    float multiplier;
};

struct SpeedLimit
{  
    // Define SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(SpeedLimit)

    // Indicates the frame_id.
    std_msgs::Header header;

    // Setting speed limit in percentage if true or in absolute values in false case
    bool percentage;

    // Maximum allowed speed (in percent of maximum robot speed or in m/s depending
    // on "percentage" value). When no-limit it is set to 0.0
    float speed_limit;
};

}  // namespace planning_msgs
}  // namespace commsgs
}  // namespace autonomy