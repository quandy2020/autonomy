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
 * @file safety_gate.cpp
 * @brief SafetyGate clamp / watchdog (implementation).
 */

#include "chassis/safety_gate.hpp"

#include <algorithm>
#include <cmath>

namespace autodriver {
namespace chassis {

ChassisCommand SafetyGate::ClampVelocity(const ChassisCommand& in) const {
  ChassisCommand out = in;
  if (!out.has_twist()) {
    return out;
  }
  auto* twist = out.mutable_twist();
  const LocomotionModel& m = limits_.model;

  if (!m.supports_lateral && twist->has_linear()) {
    twist->mutable_linear()->set_y(0.0);
  }

  if (m.max_linear_speed > 0.0 && twist->has_linear()) {
    twist->mutable_linear()->set_x(std::clamp(
        twist->linear().x(), -m.max_linear_speed, m.max_linear_speed));
    twist->mutable_linear()->set_y(std::clamp(
        twist->linear().y(), -m.max_linear_speed, m.max_linear_speed));
  }
  if (m.max_angular_speed > 0.0 && twist->has_angular()) {
    twist->mutable_angular()->set_z(std::clamp(
        twist->angular().z(), -m.max_angular_speed, m.max_angular_speed));
  }

  // Soft Ackermann hint: if inplace turn disallowed and |vx| tiny, kill wz.
  if (!m.supports_inplace_turn && twist->has_angular() && twist->has_linear()) {
    if (std::fabs(twist->linear().x()) < 1e-3) {
      twist->mutable_angular()->set_z(0.0);
    }
  }
  return out;
}

bool SafetyGate::WatchdogExpired(std::uint64_t now_ns,
                                 std::uint64_t last_cmd_ns) const {
  if (limits_.watchdog_ms <= 0) {
    return false;
  }
  const std::uint64_t timeout_ns =
      static_cast<std::uint64_t>(limits_.watchdog_ms) * 1'000'000ULL;
  return now_ns > last_cmd_ns && (now_ns - last_cmd_ns) > timeout_ns;
}

}  // namespace chassis
}  // namespace autodriver
