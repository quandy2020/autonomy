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
 * @file policy.hpp
 * @brief VINT adapter: language/vision policy with a goal-seeking fallback.
 */

#pragma once

#include <string_view>

#include "navigation/planning/interface/policy_interface.hpp"
#include "navigation/proto/plugins.pb.h"

namespace navigation
{
namespace planning
{
namespace vint
{

class Policy final : public PolicyInterface
{
public:
    /** @brief Construct with plugin options. */
    explicit Policy(plugins::VintPolicyOptions options);

    /** @brief Validate the plugin name/options. */
    bool Configure(const config::PolicyOptions& options) override;

    common::StatusOr<NavigationIntent> Query(
        const observation::Observation& observation,
        const observation::DepthFeature& depth_feature, const GoalSpec& goal,
        const memory::ContextView& context) override;

    /** @brief Reset cached state. */
    void Reset() override;

    /** @brief Plugin name. */
    [[nodiscard]] std::string_view Name() const override;

private:
    /** @brief Plugin options snapshot. */
    plugins::VintPolicyOptions options_;
    /** @brief True after Configure succeeds. */
    bool configured_{false};
};

}  // namespace vint
}  // namespace planning
}  // namespace navigation
