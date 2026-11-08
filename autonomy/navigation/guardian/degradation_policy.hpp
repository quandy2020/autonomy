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
 * @file degradation_policy.hpp
 * @brief Maps module health to a degradation mode.
 */

#pragma once

#include "navigation/proto/config.pb.h"
#include "navigation/proto/lifecycle.pb.h"

namespace navigation
{
namespace guardian
{

/** @brief Aggregated module health used for mode selection. */
struct ModuleHealth
{
    /** @brief Navigation heartbeat healthy. */
    bool navigation_ok{true};
    /** @brief Safety heartbeat healthy. */
    bool safety_ok{true};
    /** @brief Behavior heartbeat healthy. */
    bool behavior_ok{true};
    /** @brief Any fatal-class fault present. */
    bool fatal{false};
};

class DegradationPolicy
{
public:
    /** @brief Configure from guardian options. */
    void Configure(const config::GuardianConfig& configuration);

    /** @brief Map health to a degradation mode. */
    [[nodiscard]] lifecycle::DegradationMode Evaluate(const ModuleHealth& health) const;

private:
    /** @brief Whether navigation loss may fall back to reflex-only. */
    bool allow_reflex_only_{true};
};

}  // namespace guardian
}  // namespace navigation
