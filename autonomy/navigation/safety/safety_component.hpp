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
 * @file safety_component.hpp
 * @brief autolink timer component hosting the safety chain.
 */

#pragma once

#include <memory>

#include "autolink/component/timer_component.hpp"
#include "automsgs/msgs/geometry_msgs/twist_stamped.pb.h"
#include "automsgs/msgs/sensor_msgs/camera_info.pb.h"
#include "autonomy/common/macros.hpp"
#include "navigation/proto/lifecycle.pb.h"
#include "navigation/safety/safety_shield_implementation.hpp"

namespace navigation
{
namespace safety
{

class SafetyComponent final : public autolink::TimerComponent
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS_NOT_COPYABLE(SafetyComponent)

    SafetyComponent();

    ~SafetyComponent() override;

    /** @brief Release readers/writers. */
    void Clear() override;

protected:
    /** @brief Load configuration and create endpoints. */
    bool Init() override;

private:
    /** @brief Run one safety cycle. */
    bool Proc() override;

    /** @brief Three-stage safety chain. */
    SafetyShieldImplementation shield_;
    /** @brief /navigation/cmd_vel_raw reader. */
    std::shared_ptr<autolink::Reader<automsgs::msgs::geometry_msgs::Twist>>
        command_reader_{nullptr};
    /** @brief /cmd_vel writer (autosim expects TwistStamped). */
    std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::TwistStamped>>
        command_writer_{nullptr};
    /** @brief Depth image reader. */
    std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::Image>>
        depth_reader_{nullptr};
    /** @brief Odometry reader. */
    std::shared_ptr<autolink::Reader<automsgs::msgs::nav_msgs::Odometry>>
        odometry_reader_{nullptr};
    /** @brief Camera calibration reader. */
    std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::CameraInfo>>
        camera_info_reader_{nullptr};
    /** @brief /safety/heartbeat writer. */
    std::shared_ptr<autolink::Writer<lifecycle::Heartbeat>> heartbeat_writer_{nullptr};
    /** @brief Monotonic heartbeat sequence. */
    std::uint64_t heartbeat_sequence_{0};
};

}  // namespace safety
}  // namespace navigation
