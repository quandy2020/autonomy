/*
 * Copyright 2026 The Navigation Authors (navigation)
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
 * @file observation_builder.hpp
 * @brief Fuses sensor channels into one Observation with validity bits.
 */

#pragma once

#include <cstdint>

#include "automsgs/msgs/geometry_msgs/transform_stamped.pb.h"
#include "navigation/proto/observation.pb.h"

namespace navigation
{
namespace perception
{

/** @brief Non-owning sensor pointers for one fused observation. */
struct ObservationInputs
{
    /** @brief Depth image; required. */
    const automsgs::msgs::sensor_msgs::Image* depth{nullptr};
    /** @brief Camera calibration; optional. */
    const automsgs::msgs::sensor_msgs::CameraInfo* camera_info{nullptr};
    /** @brief Odometry sample; optional. */
    const automsgs::msgs::nav_msgs::Odometry* odom{nullptr};
    /** @brief IMU sample; optional. */
    const automsgs::msgs::sensor_msgs::Imu* imu{nullptr};
    /** @brief Optional occupancy grid; validated independently of depth. */
    const automsgs::msgs::map_msgs::OccupancyGrid* map{nullptr};
    /** @brief Latest ``map → odom`` from ``/tf``; used to express odom in map. */
    const automsgs::msgs::geometry_msgs::TransformStamped* map_to_odom{nullptr};
    /** @brief Result of the TF lookup for this frame. */
    bool tf_valid{false};
};

class ObservationBuilder
{
public:
    /**
     * @brief Build an Observation.
     *
     * @param[in] inputs Sensor pointers; depth is required.
     * @param[out] observation Fused message with validity bits.
     * @return false for null/empty depth or malformed input.
     */
    bool Build(const ObservationInputs& inputs, observation::Observation* observation);

    /** @brief Reset the sequence counter. */
    void Reset();

    /** @brief Last assigned sequence number. */
    [[nodiscard]] std::uint64_t LastSequence() const;

private:
    /** @brief Monotonic observation sequence. */
    std::uint64_t sequence_{0};
};

}  // namespace perception
}  // namespace navigation
