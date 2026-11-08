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

#include "navigation/planning/planning_options.hpp"

#include <cmath>
#include <utility>

#include "navigation/common/configuration_loader.hpp"

namespace navigation
{
namespace planning
{

PlanningOptions::PlanningOptions(config::NavigationConfig configuration)
    : configuration_(std::move(configuration))
{
}

const config::NavigationConfig& PlanningOptions::Configuration() const
{
    return configuration_;
}

bool PlanningOptions::Validate(std::string* error) const
{
    const auto fail = [error](const char* message) {
        if (error != nullptr)
        {
            *error = message;
        }
        return false;
    };
    if (configuration_.policy().name().empty())
    {
        return fail("policy.name is empty");
    }
    if (!configuration_.policy().has_vntpa())
    {
        return fail("policy.vntpa options are required");
    }
    const auto& vntpa = configuration_.policy().vntpa();
    if (vntpa.action_chunk_length() == 0)
    {
        return fail("action_chunk_length must be positive");
    }
    if (vntpa.execute_length() == 0 ||
        vntpa.execute_length() > vntpa.action_chunk_length())
    {
        return fail("execute_length must be in [1, action_chunk_length]");
    }
    if (!std::isfinite(vntpa.decision_rate_hz()) || vntpa.decision_rate_hz() <= 0.0)
    {
        return fail("decision_rate_hz must be positive");
    }
    if (!std::isfinite(vntpa.action_dt_s()) || vntpa.action_dt_s() <= 0.0)
    {
        return fail("action_dt_s must be positive");
    }
    if (configuration_.scenario().default_scenario().empty())
    {
        return fail("scenario.default_scenario is empty");
    }
    return true;
}

common::StatusOr<PlanningOptions> PlanningOptions::Load(const std::string& path)
{
    config::NavigationConfig configuration;
    const common::Status status = common::LoadTextProto(path, &configuration);
    if (!status.Ok())
    {
        return status;
    }
    PlanningOptions options(std::move(configuration));
    std::string error;
    if (!options.Validate(&error))
    {
        return common::Status::Configuration(error);
    }
    return options;
}

}  // namespace planning
}  // namespace navigation
