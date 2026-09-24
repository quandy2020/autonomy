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
 * @brief Bell-shaped ramp from the current command to a new target.
 *
 * Each Update() emits one step. With acc_limit <= 0 the target is returned
 * unchanged. Range is inclusive.
 */
class CommandRamp {
 public:
  void Configure(double min_cmd, double max_cmd, double acc_limit, double dt) {
    min_cmd_ = min_cmd;
    max_cmd_ = max_cmd;
    acc_limit_ = acc_limit;
    dt_ = dt > 1e-4 ? dt : 0.02;
    last_target_ = 0.0;
    const int horizon = std::max(2, static_cast<int>(1.0 / dt_));
    preview_.assign(static_cast<std::size_t>(horizon), 0.0);
  }

  double Update(double cmd) {
    cmd = std::clamp(cmd, min_cmd_, max_cmd_);
    if (acc_limit_ <= 0.0) {
      return cmd;
    }
    if (std::fabs(cmd - last_target_) > 1e-8) {
      Rebuild(cmd);
    }
    last_target_ = cmd;
    if (preview_.empty()) {
      return 0.0;
    }
    double out = preview_.front();
    preview_.erase(preview_.begin());
    preview_.push_back(preview_.empty() ? out : preview_.back());
    if (std::fabs(out) < 1e-3) {
      out = 0.0;
    }
    return out;
  }

 private:
  void Rebuild(double target) {
    const double current = preview_.empty() ? 0.0 : preview_.front();
    const double delta = target - current;
    const double abs_delta = std::fabs(delta);
    int steps = 2;
    if (abs_delta >= 1e-8) {
      steps = static_cast<int>(std::ceil(abs_delta / (acc_limit_ * dt_)));
      if (steps < 2) {
        steps = 2;
      }
    }
    double bell_sum = 0.0;
    std::vector<double> bell(static_cast<std::size_t>(steps));
    for (int i = 0; i < steps; ++i) {
      const double t =
          steps == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(steps - 1);
      bell[static_cast<std::size_t>(i)] = 0.5 * (1.0 - std::cos(kPi * t));
      bell_sum += bell[static_cast<std::size_t>(i)];
    }
    std::vector<double> next;
    next.reserve(static_cast<std::size_t>(steps));
    double value = current;
    for (int i = 0; i < steps; ++i) {
      const double share = bell_sum > 0.0 ? bell[static_cast<std::size_t>(i)] / bell_sum : 0.0;
      value = std::clamp(value + share * delta, min_cmd_, max_cmd_);
      if (std::fabs(value) < 1e-3) {
        value = 0.0;
      }
      next.push_back(value);
    }
    const int horizon = std::max(steps, static_cast<int>(1.0 / dt_));
    while (static_cast<int>(next.size()) < horizon) {
      next.push_back(std::clamp(target, min_cmd_, max_cmd_));
    }
    preview_ = std::move(next);
  }

  static constexpr double kPi = 3.14159265358979323846;
  double min_cmd_ = -1.0;
  double max_cmd_ = 1.0;
  double acc_limit_ = 0.0;
  double dt_ = 0.02;
  double last_target_ = 0.0;
  std::vector<double> preview_;
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
