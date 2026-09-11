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
 * @file locomotion_model.cpp
 * @brief LocomotionType parse / defaults (implementation).
 */

#include "chassis/locomotion_model.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace autodriver {
namespace chassis {
namespace {

std::string ToLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

}  // namespace

LocomotionType ParseLocomotionType(const std::string& text) {
  const std::string t = ToLower(text);
  if (t.empty() || t == "differential" || t == "diff" || t == "diff_drive" ||
      t == "tracked") {
    return LocomotionType::kDifferential;
  }
  if (t == "omni" || t == "omnidirectional" || t == "holonomic" ||
      t == "mecanum") {
    return LocomotionType::kOmnidirectional;
  }
  if (t == "ackermann" || t == "car" || t == "bicycle") {
    return LocomotionType::kAckermann;
  }
  if (t == "legged" || t == "quadruped" || t == "biped" || t == "leg") {
    return LocomotionType::kLegged;
  }
  if (t == "wheel_legged" || t == "wheel-legged" || t == "wheel_leg" ||
      t == "hybrid") {
    return LocomotionType::kWheelLegged;
  }
  if (t == "humanoid" || t == "biped_humanoid") {
    return LocomotionType::kHumanoid;
  }
  return LocomotionType::kUnknown;
}

const char* LocomotionTypeToString(LocomotionType type) {
  switch (type) {
    case LocomotionType::kDifferential:
      return "differential";
    case LocomotionType::kOmnidirectional:
      return "omnidirectional";
    case LocomotionType::kAckermann:
      return "ackermann";
    case LocomotionType::kLegged:
      return "legged";
    case LocomotionType::kWheelLegged:
      return "wheel_legged";
    case LocomotionType::kHumanoid:
      return "humanoid";
    case LocomotionType::kUnknown:
    default:
      return "unknown";
  }
}

void ApplyLocomotionTypeDefaults(LocomotionModel* model) {
  if (model == nullptr) {
    return;
  }
  switch (model->type) {
    case LocomotionType::kOmnidirectional:
      model->supports_lateral = true;
      model->supports_inplace_turn = true;
      break;
    case LocomotionType::kAckermann:
      model->supports_lateral = false;
      model->supports_inplace_turn = false;
      break;
    case LocomotionType::kDifferential:
    case LocomotionType::kLegged:
    case LocomotionType::kWheelLegged:
    case LocomotionType::kHumanoid:
      model->supports_lateral = false;
      model->supports_inplace_turn = true;
      break;
    case LocomotionType::kUnknown:
    default:
      break;
  }
}

}  // namespace chassis
}  // namespace autodriver
