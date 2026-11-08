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

#include "navigation/guardian/degradation_policy.hpp"

namespace navigation
{
namespace guardian
{

void DegradationPolicy::Configure(const config::GuardianConfig& configuration)
{
    allow_reflex_only_ = configuration.allow_reflex_only();
}

lifecycle::DegradationMode DegradationPolicy::Evaluate(const ModuleHealth& health) const
{
    if (health.fatal || !health.safety_ok)
    {
        return lifecycle::DEGRADATION_MODE_SAFE_STOP;
    }
    if (!health.navigation_ok || !health.behavior_ok)
    {
        return allow_reflex_only_ ? lifecycle::DEGRADATION_MODE_REFLEX_ONLY
                                  : lifecycle::DEGRADATION_MODE_SAFE_STOP;
    }
    return lifecycle::DEGRADATION_MODE_NORMAL;
}

}  // namespace guardian
}  // namespace navigation
