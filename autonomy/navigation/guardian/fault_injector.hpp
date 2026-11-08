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
 * @file fault_injector.hpp
 * @brief Test-only fault switches for the six injection classes.
 */

#pragma once

#include <array>
#include <cstddef>

namespace navigation
{
namespace guardian
{

/** @brief Six test-only fault classes. */
enum class FaultKind
{
    kHeartbeatLoss = 0,  ///< Stop navigation heartbeats.
    kStaleDepth,         ///< Freeze depth timestamps.
    kTfDisconnect,       ///< Remove map->camera transform.
    kModelFailure,       ///< Make inference fail.
    kMemoryCorruption,   ///< Corrupt snapshot data.
    kSafetyTimeout,      ///< Inject a safety-loop stall.
    kCount,              ///< Sentinel; not injectable.
};

class FaultInjector
{
public:
    /** @brief Enable one injection. */
    void Enable(FaultKind kind);
    /** @brief Disable one injection. */
    void Disable(FaultKind kind);
    /** @brief True when the injection is active. */
    [[nodiscard]] bool IsEnabled(FaultKind kind) const;
    /** @brief Disable all injections. */
    void Reset();

private:
    /** @brief One flag per fault kind. */
    std::array<bool, static_cast<std::size_t>(FaultKind::kCount)> enabled_{};
};

}  // namespace guardian
}  // namespace navigation
