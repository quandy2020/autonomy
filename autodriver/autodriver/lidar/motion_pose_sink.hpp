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
 * @brief Optional pose sink for lidar motion compensation (PushPose / PoseLookup).
 */

#ifndef AUTODRIVER_LIDAR_MOTION_POSE_SINK_HPP_
#define AUTODRIVER_LIDAR_MOTION_POSE_SINK_HPP_

#include <cstdint>
#include <memory>

#include <Eigen/Geometry>

#include "autodriver/lidar/motion_compensator.hpp"
#include "autodriver/lidar/pose_buffer.hpp"

namespace autodriver {
namespace lidar {

/**
 * @class autodriver::lidar::MotionPoseSink
 * @brief Localization / odometry feeds world←lidar poses into compensators.
 *
 * VelodyneUdpDriver / HesaiUdpDriver implement this when
 * `enable_compensator=true`. SensorManager::PushLidarPose dynamic_casts the
 * attached driver to this interface.
 */
class MotionPoseSink {
public:
    virtual ~MotionPoseSink() = default;

    /** Append a stamped pose (nanoseconds) to the built-in PoseBuffer. */
    virtual void PushPose(std::uint64_t time_ns,
                          const Eigen::Affine3d& pose) = 0;

    /** Replace the MotionCompensator pose source (advanced). */
    virtual void SetPoseLookup(PoseLookup lookup) = 0;

    /** Shared PoseBuffer when the driver owns one; else nullptr. */
    virtual std::shared_ptr<PoseBuffer> pose_buffer() const = 0;
};

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_MOTION_POSE_SINK_HPP_
