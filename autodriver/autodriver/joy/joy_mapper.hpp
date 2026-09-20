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
 * @file joy_mapper.hpp
 * @brief Pure differential teleop mapping (deadzone, enable, scale).
 */

#ifndef AUTODRIVER_JOY_JOY_MAPPER_HPP_
#define AUTODRIVER_JOY_JOY_MAPPER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autodriver {
namespace joy {

/**
 * @struct autodriver::joy::DiffTwist
 * @brief Planar differential command (body frame).
 */
struct DiffTwist {
  double linear_x = 0.0;
  double angular_z = 0.0;
};

/**
 * @brief Apply symmetric deadzone then rescale remaining range to [-1, 1].
 * @param[in] value Raw axis in [-1, 1].
 * @param[in] deadzone Deadzone magnitude in [0, 1).
 */
inline float ApplyDeadzone(float value, float deadzone) {
  const float dz = std::clamp(deadzone, 0.0f, 0.99f);
  const float abs_v = std::fabs(value);
  if (abs_v <= dz) {
    return 0.0f;
  }
  const float sign = value >= 0.0f ? 1.0f : -1.0f;
  return sign * (abs_v - dz) / (1.0f - dz);
}

/**
 * @brief Map joystick axes/buttons to differential Twist.
 *
 * When @p require_enable is true and the enable button is not pressed,
 * returns zero twist. Axis values outside range are treated as 0.
 *
 * @param[in] axes Axis vector (typically [-1, 1]).
 * @param[in] buttons Button vector (0/1).
 * @param[in] linear_axis Index for forward/back.
 * @param[in] angular_axis Index for yaw rate.
 * @param[in] invert_linear When true, negate linear axis (common for Y-up pads).
 * @param[in] invert_angular When true, negate angular axis.
 * @param[in] deadzone Axis deadzone.
 * @param[in] max_linear Scale for linear.x (m/s).
 * @param[in] max_angular Scale for angular.z (rad/s).
 * @param[in] require_enable Require enable button held.
 * @param[in] enable_button Button index that arms motion.
 */
inline DiffTwist MapDifferential(const std::vector<float>& axes,
                                 const std::vector<int32_t>& buttons,
                                 int linear_axis, int angular_axis,
                                 bool invert_linear, bool invert_angular,
                                 float deadzone, double max_linear,
                                 double max_angular, bool require_enable,
                                 int enable_button) {
  DiffTwist out;
  if (require_enable) {
    if (enable_button < 0 ||
        static_cast<std::size_t>(enable_button) >= buttons.size() ||
        buttons[static_cast<std::size_t>(enable_button)] == 0) {
      return out;
    }
  }

  float lin = 0.0f;
  float ang = 0.0f;
  if (linear_axis >= 0 &&
      static_cast<std::size_t>(linear_axis) < axes.size()) {
    lin = ApplyDeadzone(axes[static_cast<std::size_t>(linear_axis)], deadzone);
  }
  if (angular_axis >= 0 &&
      static_cast<std::size_t>(angular_axis) < axes.size()) {
    ang =
        ApplyDeadzone(axes[static_cast<std::size_t>(angular_axis)], deadzone);
  }
  if (invert_linear) {
    lin = -lin;
  }
  if (invert_angular) {
    ang = -ang;
  }
  out.linear_x = static_cast<double>(lin) * max_linear;
  out.angular_z = static_cast<double>(ang) * max_angular;
  return out;
}

}  // namespace joy
}  // namespace autodriver

#endif  // AUTODRIVER_JOY_JOY_MAPPER_HPP_
