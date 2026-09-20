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
 * @file kinematics.hpp
 * @brief JetAuto mecanum / differential inverse kinematics (m/s → motor rps).
 *
 * Layout (Hiwonder mecanum.py):
 *   motor1 |↑| motor3
 *   motor2 | | motor4
 * Motor 3/4 signs flipped before send (matching official SDK).
 */

#ifndef AUTODRIVER_CHASSIS_JETAUTO_KINEMATICS_HPP_
#define AUTODRIVER_CHASSIS_JETAUTO_KINEMATICS_HPP_

#include <array>
#include <string>

namespace autodriver {
namespace chassis {
namespace jetauto {

/** @brief JetAuto wheel geometry (defaults match JetAuto Orin docs). */
struct ChassisGeometry {
  double wheelbase = 0.216;       ///< Longitudinal axle distance L (m).
  double track_width = 0.195;     ///< Lateral track H (m).
  double wheel_diameter = 0.097;  ///< Wheel diameter (m).
};

/** @brief Drive model selected by YAML drive_mode / locomotion. */
enum class DriveMode {
  kMecanum = 0,       ///< Holonomic (vx, vy, wz).
  kDifferential = 1,  ///< Planar (vx, wz); vy forced to 0.
};

/**
 * @brief Parse drive_mode string (mecanum/omni/differential/diff).
 * @return Parsed mode; defaults to mecanum when unrecognized/empty and
 *         @p fallback_differential is false.
 */
DriveMode ParseDriveMode(const std::string& text, bool fallback_differential);

/**
 * @brief Convert body-frame twist to four motor speeds (rps).
 * @param[in] linear_x Body vx (m/s).
 * @param[in] linear_y Body vy (m/s); ignored in differential mode.
 * @param[in] angular_z Body yaw rate (rad/s).
 * @param[in] geometry Wheelbase / track / diameter.
 * @param[in] mode Mecanum or differential.
 * @param[in] max_rps Soft clamp per motor; <=0 disables.
 * @return Motor rps for ids 1..4 (index 0 → motor 1).
 */
std::array<float, 4> TwistToMotorRps(double linear_x, double linear_y,
                                     double angular_z,
                                     const ChassisGeometry& geometry,
                                     DriveMode mode, double max_rps = 0.0);

/**
 * @brief Convert tangential wheel speed (m/s) to revolutions per second.
 */
double SpeedToRps(double speed_mps, double wheel_diameter);

}  // namespace jetauto
}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_JETAUTO_KINEMATICS_HPP_
