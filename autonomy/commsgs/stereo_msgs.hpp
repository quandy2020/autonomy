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
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"


namespace autonomy {
namespace commsgs {
namespace stereo_msgs {

struct DisparityImage 
{
    // Separate header for compatibility with current TimeSynchronizer.
    // Likely to be removed in a later release, use image.header instead.
    std_msgs::Header header;

    // Floating point disparity image. The disparities are pre-adjusted for any
    // x-offset between the principal points of the two cameras (in the case
    // that they are verged). That is: d = x_l - x_r - (cx_l - cx_r)
    sensor_msgs::Image image;

    // Stereo geometry. For disparity d, the depth from the camera is Z = fT/d.
    float f; // Focal length, pixels
    float t; // Baseline, world units

    // Subwindow of (potentially) valid disparity values.
    sensor_msgs::RegionOfInterest valid_window;

    // The range of disparities searched.
    // In the disparity image, any disparity less than min_disparity is invalid.
    // The disparity search range defines the horopter, or 3D volume that the
    // stereo algorithm can "see". Points with Z outside of:
    //     Z_min = fT / max_disparity
    //     Z_max = fT / min_disparity
    // could not be found.
    float min_disparity;
    float max_disparity;

    // Smallest allowed disparity increment. The smallest achievable depth range
    // resolution is delta_Z = (Z^2/fT)*delta_d.
    float delta_d;
};

}  // namespace stereo_msgs
}  // namespace commsgs
}  // namespace autonomy