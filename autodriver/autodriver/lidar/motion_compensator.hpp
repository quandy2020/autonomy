/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief Lidar motion compensation for PointCloud2 (xyz+i+t).
 */

#ifndef AUTODRIVER_LIDAR_MOTION_COMPENSATOR_HPP_
#define AUTODRIVER_LIDAR_MOTION_COMPENSATOR_HPP_

#include <cstdint>
#include <functional>
#include <string>

#include <Eigen/Geometry>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

namespace autodriver {
namespace lidar {

/**
 * @brief Looks up world←lidar pose at an absolute time (nanoseconds).
 * @return False when no pose is available for @p time_ns.
 */
using PoseLookup = std::function<bool(
    std::uint64_t time_ns, const std::string& child_frame,
    Eigen::Affine3d* pose)>;

/** Options for MotionCompensator (world frame id). */
struct CompensatorOptions {
    std::string world_frame_id = "world";
};

/**
 * @class autodriver::lidar::MotionCompensator
 * @brief Moves each point to the scan-end lidar frame using pose interpolation.
 *
 * Requires PointCloud2 fields: `x,y,z` (float32), `intensity` (float32),
 * `timestamp` (float64 nanoseconds).
 */
class MotionCompensator {
public:
    explicit MotionCompensator(CompensatorOptions options = {});

    /** Install the pose source used by Compensate(). */
    void SetPoseLookup(PoseLookup lookup);

    /**
     * @brief Compensates in-place when possible; otherwise writes to @p out.
     * @return False if pose lookup fails or cloud lacks timestamps.
     */
    bool Compensate(const automsgs::msgs::sensor_msgs::PointCloud2& in,
                    automsgs::msgs::sensor_msgs::PointCloud2* out) const;

private:
    CompensatorOptions options_;
    PoseLookup lookup_;
};

/**
 * @brief Test helper: linear interpolation between two poses over [t_min,t_max].
 */
PoseLookup MakeLinearPoseLookup(std::uint64_t t_min, std::uint64_t t_max,
                                const Eigen::Affine3d& pose_min,
                                const Eigen::Affine3d& pose_max);

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_MOTION_COMPENSATOR_HPP_
