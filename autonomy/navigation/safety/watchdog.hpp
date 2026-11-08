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
 * @file watchdog.hpp
 * @brief Cooperative deadline watchdog for the safety loop.
 */

#pragma once

#include <chrono>
#include <cstdint>

namespace navigation
{
namespace safety
{

/** @brief Measures elapsed time against a fixed budget. */
class Watchdog
{
public:
    /** @brief Start a budget in milliseconds. */
    explicit Watchdog(std::int64_t deadline_ms);

    /** @brief True when elapsed time exceeds the budget. */
    [[nodiscard]] bool Expired() const;

    /** @brief Elapsed milliseconds since construction. */
    [[nodiscard]] double ElapsedMs() const;

    /** @brief Configured deadline in milliseconds. */
    [[nodiscard]] std::int64_t DeadlineMs() const;

private:
    /** @brief Budget start time. */
    std::chrono::steady_clock::time_point start_;
    /** @brief Budget in milliseconds. */
    std::int64_t deadline_ms_{50};
};

}  // namespace safety
}  // namespace navigation
