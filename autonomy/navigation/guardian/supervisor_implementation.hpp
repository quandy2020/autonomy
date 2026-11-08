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
 * @file supervisor_implementation.hpp
 * @brief Heartbeat monitor + degradation policy aggregation.
 */

#pragma once

#include "navigation/guardian/degradation_policy.hpp"
#include "navigation/guardian/heartbeat_monitor.hpp"
#include "navigation/guardian/supervisor.hpp"

namespace navigation
{
namespace guardian
{

class SupervisorImplementation final : public Supervisor
{
public:
    /** @brief Configure the monitor and policy. */
    bool Configure(const config::GuardianConfig& configuration) override;

    /** @brief Ingest heartbeats and return the mode. */
    lifecycle::DegradationMode Evaluate(
        const std::vector<lifecycle::Heartbeat>& heartbeats, std::int64_t now_ns) override;

    /** @brief Clear heartbeat history. */
    void Reset() override;

private:
    /** @brief Per-module freshness tracker. */
    HeartbeatMonitor monitor_;
    /** @brief Health -> mode mapping. */
    DegradationPolicy policy_;
    /** @brief Configuration snapshot. */
    config::GuardianConfig configuration_;
};

}  // namespace guardian
}  // namespace navigation
