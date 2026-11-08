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

#include "navigation/safety/watchdog.hpp"

namespace navigation
{
namespace safety
{

Watchdog::Watchdog(std::int64_t deadline_ms)
    : start_(std::chrono::steady_clock::now()), deadline_ms_(deadline_ms)
{
}

bool Watchdog::Expired() const
{
    return ElapsedMs() > static_cast<double>(deadline_ms_);
}

double Watchdog::ElapsedMs() const
{
    const auto elapsed = std::chrono::steady_clock::now() - start_;
    return std::chrono::duration<double, std::milli>(elapsed).count();
}

std::int64_t Watchdog::DeadlineMs() const
{
    return deadline_ms_;
}

}  // namespace safety
}  // namespace navigation
