/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file driver.hpp
 * @brief GENISOM 钢镚 L1-W (ZSL-1W) ChassisDriver — full HighLevel surface.
 *
 * Covers mc_sdk::zsl_1w::HighLevel (autodriver/thirdparty/zsl1w):
 * stand/lie/passive, move/crawl/climb (+ cancel), shakeHand, rearSquat,
 * attitudeControl, and state sampling (pose/twist/battery/IMU/joints).
 */

#ifndef AUTODRIVER_CHASSIS_L1W_DRIVER_HPP_
#define AUTODRIVER_CHASSIS_L1W_DRIVER_HPP_

#include "chassis/chassis_driver.hpp"
#include "autodriver/driver_params.hpp"

namespace autodriver {
namespace chassis {

/**
 * @brief Create L1-W ChassisDriver (backend "l1w").
 *
 * Params: host, local_ip, local_port, max_vx / max_vy / max_wz,
 *         allow_lateral, auto_stand, lie_on_stop, default_gait,
 *         sample_joints, simulate.
 *
 * Mode intents (shared): stand → standUp; wheel → move(); walk → crawl().
 * Climb / specials stay L1-W tools (not shared LocomotionIntent): climb,
 * cancel_climb, crawl, move, attitude, shake_hand, rear_squat, …
 *
 * Aliases: genisom, zsibot, zsl-1w, l1-w.
 */
ChassisDriver* CreateL1wChassisDriver(const ChassisId& id,
                                      const hardware::DriverParams& params);

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_L1W_DRIVER_HPP_
