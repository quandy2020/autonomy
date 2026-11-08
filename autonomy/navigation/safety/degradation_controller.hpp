/*
 * Copyright 2026 The Navigation Authors (navigation)
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
 * @file degradation_controller.hpp
 * @brief Zero-speed and degradation results for the safety chain.
 */

#pragma once

#include <string>

#include "navigation/safety/safety_shield.hpp"

namespace navigation
{
namespace safety
{

class DegradationController
{
public:
    /** @brief Build a zero-speed result flagged as emergency. */
    static SafetyResult EmergencyStop(std::string reason);

    /** @brief Build a zero-speed result without the emergency flag. */
    static SafetyResult Zero(std::string reason);
};

}  // namespace safety
}  // namespace navigation
