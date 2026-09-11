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
 * @file capability.cpp
 * @brief CapabilityProfile JSON serializer (implementation).
 */

#include "chassis/capability.hpp"

#include <sstream>

namespace autodriver {
namespace chassis {
namespace {

std::string EscapeJson(const std::string& in) {
  std::string out;
  out.reserve(in.size() + 8);
  for (char c : in) {
    switch (c) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

}  // namespace

std::string CapabilityProfileToJson(const CapabilityProfile& profile) {
  std::ostringstream oss;
  oss << '{'
      << "\"chassis_id\":\"" << EscapeJson(profile.chassis_id) << "\","
      << "\"backend\":\"" << EscapeJson(profile.backend) << "\","
      << "\"locomotion\":\""
      << LocomotionTypeToString(profile.locomotion.type) << "\","
      << "\"max_linear_speed\":" << profile.locomotion.max_linear_speed << ','
      << "\"max_angular_speed\":" << profile.locomotion.max_angular_speed << ','
      << "\"max_linear_accel\":" << profile.locomotion.max_linear_accel << ','
      << "\"min_turning_radius\":" << profile.locomotion.min_turning_radius
      << ','
      << "\"supports_lateral\":"
      << (profile.locomotion.supports_lateral ? "true" : "false") << ','
      << "\"supports_inplace_turn\":"
      << (profile.locomotion.supports_inplace_turn ? "true" : "false") << ','
      << "\"has_dock\":" << (profile.has_dock ? "true" : "false") << ','
      << "\"has_tool_channel\":" << (profile.has_tool_channel ? "true" : "false")
      << ','
      << "\"has_joint_bypass\":"
      << (profile.has_joint_bypass ? "true" : "false") << ','
      << "\"require_arm\":" << (profile.require_arm ? "true" : "false") << ','
      << "\"tools\":[";
  for (std::size_t i = 0; i < profile.tools.size(); ++i) {
    if (i > 0) {
      oss << ',';
    }
    oss << '"' << EscapeJson(profile.tools[i]) << '"';
  }
  oss << "]}";
  return oss.str();
}

}  // namespace chassis
}  // namespace autodriver
