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
 * @file safety_shield_implementation.hpp
 * @brief Concrete three-stage safety chain.
 */

#pragma once

#include <cstdint>

#include "navigation/safety/aeb_velocity_sampler.hpp"
#include "navigation/safety/braking_model.hpp"
#include "navigation/safety/cbf_filter.hpp"
#include "navigation/safety/reflex_policy_arbiter.hpp"
#include "navigation/safety/safety_shield.hpp"
#include "navigation/safety/velocity_feasibility_filter.hpp"

namespace navigation
{
namespace safety
{

class SafetyShieldImplementation final : public SafetyShield
{
public:
    SafetyShieldImplementation() = default;

    /** @brief Configure from SafetyConfig. */
    bool Configure(const config::SafetyConfig& configuration) override;

    bool Configure(const common::PlatformProfile& profile,
                   const config::SafetyChainOptions& chain,
                   const plugins::ReactiveAvoiderOptions& reflex) override;

    SafetyResult Process(const common::Twist2D& candidate,
                         const ReflexObservation& observation,
                         const common::Twist2D& measured, std::int64_t now_ns,
                         double dt_s) override;

    [[nodiscard]] bool IsSafe(const common::Twist2D& command,
                              const ReflexObservation& observation) const override;

    /** @brief Reset feasibility history. */
    void Reset() override;

private:
    /** @brief Validate finite inputs, timestamp freshness and dt. */
    bool ValidInputs(const common::Twist2D& candidate, const common::Twist2D& measured,
                     const ReflexObservation& observation, std::int64_t now_ns,
                     double dt_s) const;

    /** @brief Validated platform limits. */
    common::PlatformProfile profile_;
    /** @brief Three-stage chain parameters. */
    config::SafetyChainOptions chain_;
    /** @brief Near-field reflex parameters. */
    plugins::ReactiveAvoiderOptions reflex_options_;
    /** @brief CBF projection stage. */
    CbfFilter cbf_;
    /** @brief AEB fallback sampler. */
    AebVelocitySampler aeb_;
    /** @brief Acceleration/jerk limiter. */
    VelocityFeasibilityFilter feasibility_;
    /** @brief Policy/reflex arbitration stage. */
    ReflexPolicyArbiter reflex_arbiter_;
    /** @brief Braking-distance model. */
    BrakingModel braking_model_{1.0, 0.15};
    /** @brief True after successful Configure. */
    bool configured_{false};
};

}  // namespace safety
}  // namespace navigation
