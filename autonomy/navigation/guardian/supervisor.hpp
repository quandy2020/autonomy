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
 * @file supervisor.hpp
 * @brief Aggregates heartbeats into a degradation mode.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "navigation/proto/config.pb.h"
#include "navigation/proto/lifecycle.pb.h"

namespace navigation
{
namespace guardian
{

class Supervisor
{
public:
    virtual ~Supervisor() = default;

    /** @brief Configure heartbeat windows and degradation policy. */
    virtual bool Configure(const config::GuardianConfig& configuration) = 0;

    /**
     * @brief Update heartbeats and evaluate the degradation mode.
     *
     * @param[in] heartbeats Latest heartbeats (0..3 entries).
     * @param[in] now_ns Monotonic time.
     */
    virtual lifecycle::DegradationMode Evaluate(
        const std::vector<lifecycle::Heartbeat>& heartbeats, std::int64_t now_ns) = 0;

    /** @brief Drop all heartbeat state. */
    virtual void Reset() = 0;
};

}  // namespace guardian
}  // namespace navigation
