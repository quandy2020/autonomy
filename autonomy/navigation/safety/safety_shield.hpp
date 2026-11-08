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
 * @file safety_shield.hpp
 * @brief Three-stage safety chain boundary.
 */

#pragma once

#include <cstdint>
#include <string>

#include "navigation/common/platform_profile.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/proto/config.pb.h"
#include "navigation/proto/navigation.pb.h"
#include "navigation/proto/plugins.pb.h"

namespace navigation
{
namespace safety
{

/** @brief Output of one safety-chain evaluation. */
/** @brief Output of one safety-chain evaluation. */
struct SafetyResult
{
    /** @brief Safe command; zero on any failure. */
    common::Twist2D command;
    /** @brief True when the chain fell back to zero speed. */
    bool emergency{false};
    /** @brief True when the reflex layer overrode the policy direction. */
    bool reflex_intervened{false};
    /** @brief True when AEB sampling replaced an infeasible CBF projection. */
    bool aeb_intervened{false};
    /** @brief True when acceleration/jerk limiting changed the command. */
    bool feasibility_intervened{false};
    /** @brief Machine-readable reason ("ok" on success). */
    std::string reason;
};

/**
 * @brief Safety chain: validate -> reflex -> CBF/AEB -> feasibility.
 *
 * @par Threading
 * Owned by the safety process; all methods run on its timer thread.
 */
class SafetyShield
{
public:
    virtual ~SafetyShield() = default;

    /**
     * @brief Configure from file-level SafetyConfig.
     *
     * @param[in] configuration Platform preset plus chain/reflex options.
     * @return false when the profile or thresholds are invalid.
     */
    virtual bool Configure(const config::SafetyConfig& configuration) = 0;

    /**
     * @brief Configure from already-parsed values (tests/embedding).
     *
     * @param[in] profile Platform limits.
     * @param[in] chain Three-stage chain parameters.
     * @param[in] reflex Near-field reflex options.
     */
    virtual bool Configure(const common::PlatformProfile& profile,
                           const config::SafetyChainOptions& chain,
                           const plugins::ReactiveAvoiderOptions& reflex) = 0;

    /**
     * @brief Run one safety cycle.
     *
     * @param[in] candidate Policy/blender command.
     * @param[in] observation Near-field obstacles with a fresh timestamp.
     * @param[in] measured Measured velocity for acceleration limits.
     * @param[in] now_ns Monotonic time.
     * @param[in] dt_s Control period in seconds.
     * @return Safe command or a zero-speed emergency result.
     */
    virtual SafetyResult Process(const common::Twist2D& candidate,
                                 const ReflexObservation& observation,
                                 const common::Twist2D& measured, std::int64_t now_ns,
                                 double dt_s) = 0;

    /** @brief Check a command against the barrier constraints. */
    [[nodiscard]] virtual bool IsSafe(const common::Twist2D& command,
                                      const ReflexObservation& observation) const = 0;

    /** @brief Reset feasibility history. */
    virtual void Reset() = 0;
};

}  // namespace safety
}  // namespace navigation
