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
 * @file policy_interface.hpp
 * @brief End-to-end policy plugin boundary.
 */

#pragma once

#include <string_view>

#include "navigation/common/status.hpp"
#include "navigation/common/status_or.hpp"
#include "navigation/memory/context_view.hpp"
#include "navigation/proto/config.pb.h"
#include "navigation/proto/navigation.pb.h"
#include "navigation/proto/observation.pb.h"

namespace navigation
{
namespace planning
{

/**
 * @brief Policy plugin interface (VNTPA, VINT, NOMAD).
 *
 * @par Threading
 * Configure/Reset run on the startup thread; Query runs on the policy worker
 * and must not be called concurrently with itself.
 */
class PolicyInterface
{
public:
    virtual ~PolicyInterface() = default;

    /** @brief Apply plugin options; false leaves the plugin unusable. */
    virtual bool Configure(const config::PolicyOptions& options) = 0;

    /** @brief Query one navigation intent. */
    virtual common::StatusOr<NavigationIntent> Query(
        const observation::Observation& observation,
        const observation::DepthFeature& depth_feature, const GoalSpec& goal,
        const memory::ContextView& context) = 0;

    /** @brief Drop recurrent state and cached actions. */
    virtual void Reset() = 0;

    /** @brief Stable plugin name used in configuration and diagnostics. */
    [[nodiscard]] virtual std::string_view Name() const = 0;
};

}  // namespace planning
}  // namespace navigation
