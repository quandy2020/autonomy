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

/**
 * @file grid.hpp
 * @brief Rolling 2.5D safety grid for Shadow ground-person following.
 */

#ifndef AUTONOMY_PERCEPTION_SHADOW_GRID_HPP_
#define AUTONOMY_PERCEPTION_SHADOW_GRID_HPP_

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/perception/shadow/proto/shadow.pb.h"

#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/map_msgs/grid_map.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include <Eigen/Core>

#include <cstdint>
#include <string>

namespace autonomy {
namespace perception {
namespace shadow {

/**
 * @brief Fuses aligned depth into a robot-centered rolling safety map.
 *
 * The public map contains exactly elevation, variance, obstacle, and
 * traversability. Unknown cells are represented by NaN in every layer.
 */
class LocalGrid
{
public:
    /**
     * @brief Creates an unknown map from validated Shadow options.
     */
    explicit LocalGrid(const proto::ShadowOptions& options);

    /**
     * @brief Atomically fuses one aligned depth frame into the local map.
     * @param stamp_ns Acquisition timestamp in nanoseconds.
     * @param depth Aligned `16UC1` or metric `32FC1` depth image.
     * @param camera Camera intrinsics corresponding to depth.
     * @param camera_to_map Rigid transform from camera into map.
     * @param odometry Map-frame robot pose and support-plane height.
     * @param error Optional diagnostic output, cleared on entry.
     * @return True when validation, rolling, fusion, and classification finish.
     */
    bool Update(
        int64_t stamp_ns, const automsgs::msgs::sensor_msgs::Image& depth,
        const automsgs::msgs::sensor_msgs::CameraInfo& camera,
        const automsgs::msgs::geometry_msgs::TransformStamped& camera_to_map,
        const automsgs::msgs::nav_msgs::Odometry& odometry,
        std::string* error = nullptr);

    /** @brief Returns the current four-layer map. */
    const grid_map::GridMap& map() const;

    /**
     * @brief Converts the current map through the repository converter.
     * @param message Existing automsgs GridMap output.
     * @param error Optional diagnostic output, cleared on entry.
     * @return False only when message is null.
     */
    bool ToMessage(automsgs::msgs::map_msgs::GridMap* message,
                   std::string* error = nullptr) const;

    /** @brief Restores the initial unknown, zero-timestamp map. */
    void Clear();

private:
    using CountMatrix = Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic>;
    using TimestampMatrix =
        Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>;

    proto::ShadowOptions options_;
    grid_map::GridMap map_;
    CountMatrix sample_count_;
    grid_map::Matrix elevation_m2_;
    TimestampMatrix last_observed_ns_;
};

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_SHADOW_GRID_HPP_
