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

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace control {
namespace utils {

/**
 * @brief Transform a PoseStamped from one frame to another while catching exceptions
 *
 * Also returns immediately if the frames are equal.
 * @param tf Smart pointer to TFListener
 * @param frame Frame to transform the pose into
 * @param in_pose Pose to transform
 * @param out_pose Place to store the resulting transformed pose
 * @param extrapolation_fallback If true, if there is an ExtrapolationException, allow looking up the latest timestamp
 * instead.
 * @return True if successful transform
 */
bool transformPose(const TFListenerPtr tf, const std::string frame, const commsgs::geometry_msgs::PoseStamped& in_pose,
                   commsgs::geometry_msgs::PoseStamped& out_pose, const bool extrapolation_fallback = true) {
    if (in_pose.header.frame_id == frame) {
        out_pose = in_pose;
        return true;
    }

    try {
        tf->transform(in_pose, out_pose, frame);
        return true;
    } catch (tf::ExtrapolationException& ex) {
        if (!extrapolation_fallback)
            throw;
        commsgs::geometry_msgs::PoseStamped latest_in_pose;
        latest_in_pose.header.frame_id = in_pose.header.frame_id;
        latest_in_pose.pose = in_pose.pose;
        tf->transform(latest_in_pose, out_pose, frame);
        return true;
    } catch (tf::TransformException& ex) {
        AERROR << "Exception in transformPose: " << ex.what();
        return false;
    }
    return false;
}

/**
 * @brief Transform a Pose2DStamped from one frame to another while catching exceptions
 *
 * Also returns immediately if the frames are equal.
 * @param tf Smart pointer to TFListener
 * @param frame Frame to transform the pose into
 * @param in_pose Pose to transform
 * @param out_pose Place to store the resulting transformed pose
 * @param extrapolation_fallback If true, if there is an ExtrapolationException, allow looking up the latest timestamp
 * instead.
 * @return True if successful transform
 */
bool transformPose(const TFListenerPtr tf, const std::string frame,
                   const commsgs::planning_msgs::Pose2DStamped& in_pose, commsgs::planing_msgs::Pose2DStamped& out_pose,
                   const bool extrapolation_fallback = true) {
    commsgs::geometry_msgs::PoseStamped in_3d_pose = pose2DToPoseStamped(in_pose);
    commsgs::geometry_msgs::PoseStamped out_3d_pose;

    bool ret = transformPose(tf, frame, in_3d_pose, out_3d_pose, extrapolation_fallback);
    if (ret) {
        out_pose = poseStampedToPose2D(out_3d_pose);
    }
    return ret;
}

commsgs::geometry_msgs::Pose2D transformStampedPose(const TFListenerPtr tf,
                                                    const commsgs::planning_msgs::Pose2DStamped& pose,
                                                    const std::string& frame_id) {
    commsgs::planning_msgs::Pose2DStamped local_pose;
    commsgs::nav_2d_utils::transformPose(tf, frame_id, pose, local_pose);
    return local_pose.pose;
}

}  // namespace utils
}  // namespace control
}  // namespace autonomy