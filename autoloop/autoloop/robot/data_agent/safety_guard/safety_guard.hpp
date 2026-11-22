/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
/**
 * @file safety_guard.hpp
 * @brief L4 latch-only. Stays set until Clear().
 */
#include "robot/data_agent/safety_guard/safety_checker.hpp"

namespace autoloop {
namespace robot {

class SafetyGuard {
public:
    void Observe(bool estop, bool cmd_mismatch);
    void Clear();
    bool latched() const { return latched_; }

private:
    bool latched_{false};
};

}  // namespace robot
}  // namespace autoloop
