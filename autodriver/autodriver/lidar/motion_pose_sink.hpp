/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file motion_pose_sink.hpp
 * @brief Optional pose sink for lidar motion compensation (PushPose / PoseLookup).
 */

#ifndef AUTODRIVER_LIDAR_MOTION_POSE_SINK_HPP_
#define AUTODRIVER_LIDAR_MOTION_POSE_SINK_HPP_

#include <cstdint>
#include <memory>

#include <Eigen/Geometry>

#include "autodriver/lidar/motion_compensator.hpp"
#include "autodriver/lidar/pose_buffer.hpp"
#include "autolink/common/macros.hpp"

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
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(MotionPoseSink)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(MotionPoseSink)

    /**
     * @brief Default constructor (DISALLOW_COPY suppresses the implicit one).
     */
    MotionPoseSink() = default;

    /**
     * @brief Virtual destructor for polymorphic pose sinks.
     */
    virtual ~MotionPoseSink() = default;

    /**
     * @brief Append a stamped pose (nanoseconds) to the built-in PoseBuffer.
     * @param[in] time_ns Pose timestamp in nanoseconds.
     * @param[in] pose world←lidar affine transform at @p time_ns.
     */
    virtual void PushPose(std::uint64_t time_ns,
                          const Eigen::Affine3d& pose) = 0;

    /**
     * @brief Replace the MotionCompensator pose source (advanced).
     * @param[in] lookup Callable used instead of / in addition to PoseBuffer.
     */
    virtual void SetPoseLookup(PoseLookup lookup) = 0;

    /**
     * @brief Shared PoseBuffer when the driver owns one; else nullptr.
     * @return PoseBuffer shared pointer, or nullptr when compensation is off.
     */
    virtual std::shared_ptr<PoseBuffer> pose_buffer() const = 0;
};

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_MOTION_POSE_SINK_HPP_
