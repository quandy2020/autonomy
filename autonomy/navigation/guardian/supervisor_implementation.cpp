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

#include "navigation/guardian/supervisor_implementation.hpp"

namespace navigation
{
namespace guardian
{

bool SupervisorImplementation::Configure(const config::GuardianConfig& configuration)
{
    if (configuration.heartbeat_timeout_ms() == 0 ||
        configuration.heartbeat_fatal_ms() < configuration.heartbeat_timeout_ms())
    {
        return false;
    }
    configuration_ = configuration;
    monitor_.Configure(configuration.heartbeat_timeout_ms(),
                       configuration.heartbeat_fatal_ms());
    policy_.Configure(configuration);
    return true;
}

lifecycle::DegradationMode SupervisorImplementation::Evaluate(
    const std::vector<lifecycle::Heartbeat>& heartbeats, std::int64_t now_ns)
{
    for (const auto& heartbeat : heartbeats)
    {
        (void)monitor_.Update(heartbeat, now_ns);
    }
    const auto navigation = monitor_.Check("navigation", now_ns);
    const auto safety = monitor_.Check("safety", now_ns);
    const auto behavior = monitor_.Check("behavior", now_ns);

    ModuleHealth health;
    health.navigation_ok = navigation.healthy;
    health.safety_ok = safety.healthy;
    health.behavior_ok = behavior.healthy;
    health.fatal = safety.fatal;
    return policy_.Evaluate(health);
}

void SupervisorImplementation::Reset()
{
    monitor_.Reset();
}

}  // namespace guardian
}  // namespace navigation
