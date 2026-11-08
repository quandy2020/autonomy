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
 * @file diagnostics_publisher.hpp
 * @brief Per-code diagnostic rate limiting.
 */

#pragma once

#include <cstdint>
#include <unordered_map>

namespace navigation
{
namespace guardian
{

class DiagnosticsPublisher
{
public:
    /** @brief Configure per-code rate limiting. */
    void Configure(double rate_hz);

    /** @brief True when the code may be published at this time. */
    bool ShouldPublish(std::uint32_t code, std::int64_t now_ns);

    /** @brief Clear per-code timestamps. */
    void Reset();

private:
    /** @brief Minimum interval between repeats in nanoseconds. */
    std::int64_t interval_ns_{500000000};
    /** @brief Diagnostic code -> last publish time. */
    std::unordered_map<std::uint32_t, std::int64_t> last_ns_;
};

}  // namespace guardian
}  // namespace navigation
