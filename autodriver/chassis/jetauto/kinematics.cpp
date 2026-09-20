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
 * @file kinematics.cpp
 * @brief JetAuto mecanum / differential IK (implementation).
 */

#include "chassis/jetauto/kinematics.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace autodriver {
namespace chassis {
namespace jetauto {
namespace {

std::string ToLower(std::string text) {
  for (char& c : text) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return text;
}

float ClampRps(double value, double max_rps) {
  if (max_rps > 0.0) {
    value = std::clamp(value, -max_rps, max_rps);
  }
  return static_cast<float>(value);
}

}  // namespace

DriveMode ParseDriveMode(const std::string& text, bool fallback_differential) {
  const std::string lower = ToLower(text);
  if (lower.empty()) {
    return fallback_differential ? DriveMode::kDifferential
                                 : DriveMode::kMecanum;
  }
  if (lower == "differential" || lower == "diff" || lower == "diff_drive" ||
      lower == "tank") {
    return DriveMode::kDifferential;
  }
  if (lower == "mecanum" || lower == "omni" || lower == "omnidirectional" ||
      lower == "holonomic") {
    return DriveMode::kMecanum;
  }
  return fallback_differential ? DriveMode::kDifferential : DriveMode::kMecanum;
}

double SpeedToRps(double speed_mps, double wheel_diameter) {
  if (wheel_diameter <= 0.0) {
    return 0.0;
  }
  return speed_mps / (3.14159265358979323846 * wheel_diameter);
}

std::array<float, 4> TwistToMotorRps(double linear_x, double linear_y,
                                     double angular_z,
                                     const ChassisGeometry& geometry,
                                     DriveMode mode, double max_rps) {
  const double vy =
      (mode == DriveMode::kDifferential) ? 0.0 : linear_y;
  const double k =
      (geometry.wheelbase + geometry.track_width) * 0.5;
  // Official Hiwonder mecanum.py order + motor3/4 sign flip.
  const double m1 = linear_x - vy - angular_z * k;
  const double m2 = linear_x + vy - angular_z * k;
  const double m3 = linear_x + vy + angular_z * k;
  const double m4 = linear_x - vy + angular_z * k;
  return {
      ClampRps(SpeedToRps(m1, geometry.wheel_diameter), max_rps),
      ClampRps(SpeedToRps(m2, geometry.wheel_diameter), max_rps),
      ClampRps(SpeedToRps(-m3, geometry.wheel_diameter), max_rps),
      ClampRps(SpeedToRps(-m4, geometry.wheel_diameter), max_rps),
  };
}

}  // namespace jetauto
}  // namespace chassis
}  // namespace autodriver
