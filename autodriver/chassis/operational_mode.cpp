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
 * @file operational_mode.cpp
 * @brief OperationalModeController FSM (implementation).
 */

#include "chassis/operational_mode.hpp"

#include <algorithm>
#include <cctype>

namespace autodriver {
namespace chassis {
namespace {

std::string TrimLower(std::string s) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

}  // namespace

const char* OperationalModeToString(OperationalMode mode) {
  switch (mode) {
    case OperationalMode::kIdle:
      return "idle";
    case OperationalMode::kArmed:
      return "armed";
    case OperationalMode::kMoving:
      return "moving";
    case OperationalMode::kDocking:
      return "docking";
    case OperationalMode::kFault:
      return "fault";
    case OperationalMode::kEStop:
      return "estop";
    default:
      return "unknown";
  }
}

const char* LocomotionIntentToString(LocomotionIntent intent) {
  switch (intent) {
    case LocomotionIntent::kStand:
      return "stand";
    case LocomotionIntent::kWalk:
      return "walk";
    case LocomotionIntent::kWheel:
      return "wheel";
    case LocomotionIntent::kUnspecified:
    default:
      return "unspecified";
  }
}

OperationalMode ParseOperationalMode(const std::string& text) {
  const std::string t = TrimLower(text);
  if (t == "idle") {
    return OperationalMode::kIdle;
  }
  if (t == "armed") {
    return OperationalMode::kArmed;
  }
  if (t == "moving") {
    return OperationalMode::kMoving;
  }
  if (t == "docking") {
    return OperationalMode::kDocking;
  }
  if (t == "fault") {
    return OperationalMode::kFault;
  }
  if (t == "estop" || t == "e-stop" || t == "emergency_stop") {
    return OperationalMode::kEStop;
  }
  return OperationalMode::kIdle;
}

OperationalMode OperationalModeController::mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

LocomotionIntent OperationalModeController::intent() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return intent_;
}

std::string OperationalModeController::ModeString() const {
  return OperationalModeToString(mode());
}

std::string OperationalModeController::IntentString() const {
  return LocomotionIntentToString(intent());
}

bool OperationalModeController::AllowsMotion(bool require_arm) const {
  std::lock_guard<std::mutex> lock(mutex_);
  switch (mode_) {
    case OperationalMode::kArmed:
    case OperationalMode::kMoving:
    case OperationalMode::kDocking:
      return true;
    case OperationalMode::kIdle:
      return !require_arm;
    case OperationalMode::kFault:
    case OperationalMode::kEStop:
    default:
      return false;
  }
}

void OperationalModeController::NotifyMotionApplied(bool non_zero_twist) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ == OperationalMode::kEStop || mode_ == OperationalMode::kFault) {
    return;
  }
  if (non_zero_twist) {
    if (mode_ == OperationalMode::kArmed || mode_ == OperationalMode::kIdle) {
      mode_ = OperationalMode::kMoving;
    }
  } else if (mode_ == OperationalMode::kMoving) {
    mode_ = OperationalMode::kArmed;
  }
}

void OperationalModeController::NotifyFault() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != OperationalMode::kEStop) {
    mode_ = OperationalMode::kFault;
  }
}

void OperationalModeController::NotifyEStop() {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = OperationalMode::kEStop;
}

bool OperationalModeController::HandleModeCommand(const std::string& command,
                                                    std::string* error) {
  const std::string cmd = TrimLower(command);
  std::lock_guard<std::mutex> lock(mutex_);

  auto fail = [&](const char* msg) {
    if (error != nullptr) {
      *error = msg;
    }
    return false;
  };

  if (cmd.empty()) {
    return fail("empty mode command");
  }

  if (cmd == "estop" || cmd == "e-stop" || cmd == "emergency_stop") {
    mode_ = OperationalMode::kEStop;
    return true;
  }
  if (cmd == "clear_estop" || cmd == "reset_estop") {
    if (mode_ != OperationalMode::kEStop) {
      return fail("not in estop");
    }
    mode_ = OperationalMode::kIdle;
    return true;
  }
  if (cmd == "clear_fault" || cmd == "reset_fault") {
    if (mode_ != OperationalMode::kFault) {
      return fail("not in fault");
    }
    mode_ = OperationalMode::kIdle;
    return true;
  }
  if (cmd == "arm" || cmd == "enable") {
    if (mode_ == OperationalMode::kEStop || mode_ == OperationalMode::kFault) {
      return fail("clear estop/fault before arm");
    }
    mode_ = OperationalMode::kArmed;
    return true;
  }
  if (cmd == "disarm" || cmd == "disable") {
    if (mode_ == OperationalMode::kEStop) {
      return fail("clear estop before disarm");
    }
    mode_ = OperationalMode::kIdle;
    return true;
  }
  if (cmd == "dock") {
    if (mode_ == OperationalMode::kEStop || mode_ == OperationalMode::kFault) {
      return fail("cannot dock in estop/fault");
    }
    mode_ = OperationalMode::kDocking;
    return true;
  }
  if (cmd == "undock") {
    if (mode_ != OperationalMode::kDocking) {
      return fail("not docking");
    }
    mode_ = OperationalMode::kArmed;
    return true;
  }
  if (cmd == "stand") {
    intent_ = LocomotionIntent::kStand;
    return true;
  }
  if (cmd == "walk") {
    intent_ = LocomotionIntent::kWalk;
    return true;
  }
  if (cmd == "wheel") {
    intent_ = LocomotionIntent::kWheel;
    return true;
  }

  return fail("unknown mode command");
}

}  // namespace chassis
}  // namespace autodriver
