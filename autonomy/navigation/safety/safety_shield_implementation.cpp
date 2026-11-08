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

#include "navigation/safety/safety_shield_implementation.hpp"
#include "autonomy/common/helper_functions/float_comparisons.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "navigation/common/platform_registry.hpp"
#include "navigation/safety/degradation_controller.hpp"
#include "navigation/safety/watchdog.hpp"

namespace navigation
{
namespace safety
{
namespace
{

constexpr std::int64_t kMaxObservationAgeNs = 200000000;

}  // namespace

bool SafetyShieldImplementation::Configure(const config::SafetyConfig& configuration)
{
    common::PlatformProfile profile;
    const std::string& name = configuration.platform_profile();
    const std::string path = name.find('/') != std::string::npos || name.find(".pb.txt") !=
                                                                         std::string::npos
                                 ? name
                                 : "platform/" + name + ".pb.txt";
    if (!common::LoadPlatformProfile(path, &profile).Ok())
    {
        configured_ = false;
        return false;
    }
    plugins::ReactiveAvoiderOptions reflex;
    if (configuration.has_reflex() && configuration.reflex().has_reactive_avoider())
    {
        reflex = configuration.reflex().reactive_avoider();
    }
    else
    {
        reflex.set_near_field_range(2.0);
        reflex.set_reaction_time_ms(30.0);
        reflex.set_safety_margin(0.3);
        reflex.set_fov_rad(1.57);
        reflex.set_candidate_count(32);
    }
    return Configure(profile, configuration.chain(), reflex);
}

bool SafetyShieldImplementation::Configure(
    const common::PlatformProfile& profile, const config::SafetyChainOptions& chain,
    const plugins::ReactiveAvoiderOptions& reflex)
{
    if (!common::IsValid(profile) || chain.watchdog_deadline_ms() == 0 ||
        !std::isfinite(chain.cbf_alpha()) || chain.cbf_alpha() <= 0.0 ||
        !std::isfinite(chain.max_acceleration_mps2()) ||
        chain.max_acceleration_mps2() <= 0.0)
    {
        configured_ = false;
        return false;
    }
    profile_ = profile;
    chain_ = chain;
    reflex_options_ = reflex;
    if (!cbf_.Configure(profile, chain.cbf_alpha(), chain.min_clearance()) ||
        !feasibility_.Configure(profile))
    {
        configured_ = false;
        return false;
    }
    braking_model_ = BrakingModel(chain.max_acceleration_mps2(),
                                  chain.ttc_reaction_time_s());
    configured_ = true;
    return true;
}

SafetyResult SafetyShieldImplementation::Process(const common::Twist2D& candidate,
                                                 const ReflexObservation& observation,
                                                 const common::Twist2D& measured,
                                                 std::int64_t now_ns, double dt_s)
{
    Watchdog watchdog(static_cast<std::int64_t>(chain_.watchdog_deadline_ms()));
    if (!configured_ || !ValidInputs(candidate, measured, observation, now_ns, dt_s))
    {
        return DegradationController::EmergencyStop("invalid_or_stale_input");
    }

    const ReflexCommand reflex =
        reflex_arbiter_.Arbitrate(candidate, observation, reflex_options_,
                                  chain_.reflex_priority_ratio());

    std::optional<common::Twist2D> safe = cbf_.Project(reflex.velocity, observation);
    bool aeb_intervened = false;
    if (!safe.has_value())
    {
        safe = aeb_.Sample(reflex.velocity, observation, profile_, cbf_,
                           chain_.aeb_sample_count() == 0 ? 8 : chain_.aeb_sample_count());
        aeb_intervened = true;
    }
    if (!safe.has_value())
    {
        return DegradationController::EmergencyStop("infeasible");
    }

    common::Twist2D feasible;
    if (!feasibility_.Filter(*safe, measured, dt_s, &feasible))
    {
        return DegradationController::EmergencyStop("feasibility_failed");
    }

    bool feasibility_intervened =
        !autonomy::common::helper_functions::abs_eq(feasible.x(), safe->x(), 1e-6F) ||
        !autonomy::common::helper_functions::abs_eq(feasible.theta(), safe->theta(),
                                                    1e-6F);
    if (!cbf_.IsSafe(feasible, observation))
    {
        common::Twist2D decelerated = measured;
        const double delta = profile_.max_acceleration_mps2 * dt_s;
        decelerated.set_x(std::abs(decelerated.x()) <= delta ? 0.0
                                                        : decelerated.x() -
                                                              (decelerated.x() > 0.0
                                                                   ? 1.0
                                                                   : -1.0) *
                                                                  delta);
        decelerated.set_y(0.0);
        decelerated.set_theta(0.0);
        if (!cbf_.IsSafe(decelerated, observation))
        {
            return DegradationController::EmergencyStop("unsafe_after_clamp");
        }
        feasible = decelerated;
        feasibility_intervened = true;
    }

    if (watchdog.Expired())
    {
        return DegradationController::EmergencyStop("deadline");
    }

    SafetyResult result;
    result.command = feasible;
    result.aeb_intervened = aeb_intervened;
    result.reflex_intervened = reflex.intervened;
    result.feasibility_intervened = feasibility_intervened;
    result.reason = "ok";
    return result;
}

bool SafetyShieldImplementation::IsSafe(const common::Twist2D& command,
                                        const ReflexObservation& observation) const
{
    return configured_ && cbf_.IsSafe(command, observation);
}

void SafetyShieldImplementation::Reset()
{
    feasibility_.Reset();
    cbf_ = CbfFilter{};
    (void)cbf_.Configure(profile_, chain_.cbf_alpha(), chain_.min_clearance());
}

bool SafetyShieldImplementation::ValidInputs(const common::Twist2D& candidate,
                                             const common::Twist2D& measured,
                                             const ReflexObservation& observation,
                                             std::int64_t now_ns, double dt_s) const
{
    if (!common::IsFinite(candidate) || !common::IsFinite(measured) ||
        !observation.valid() || !std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > 0.5)
    {
        return false;
    }
    const std::int64_t observation_age_ns = now_ns - observation.stamp_ns();
    if (observation.stamp_ns() <= 0 || observation_age_ns < 0 ||
        observation_age_ns > kMaxObservationAgeNs)
    {
        return false;
    }
    for (const auto& obstacle : observation.obstacles())
    {
        if (!std::isfinite(obstacle.x()) || !std::isfinite(obstacle.y()) ||
            !std::isfinite(obstacle.vx_mps()) || !std::isfinite(obstacle.vy_mps()) ||
            !std::isfinite(obstacle.radius()) || obstacle.radius() < 0.0)
        {
            return false;
        }
    }
    return true;
}

}  // namespace safety
}  // namespace navigation
