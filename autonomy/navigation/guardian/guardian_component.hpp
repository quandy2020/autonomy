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
 * @file guardian_component.hpp
 * @brief autolink timer component hosting the supervisor.
 */

#pragma once

#include <memory>

#include "autolink/component/timer_component.hpp"
#include "autonomy/common/macros.hpp"
#include "navigation/guardian/supervisor_implementation.hpp"

namespace navigation
{
namespace guardian
{

class GuardianComponent final : public autolink::TimerComponent
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS_NOT_COPYABLE(GuardianComponent)

    GuardianComponent();

    ~GuardianComponent() override;

    /** @brief Clear all state. */
    void Clear() override;

protected:
    /** @brief Load configuration and create endpoints. */
    bool Init() override;

private:
    /** @brief Evaluate heartbeats and publish degradation. */
    bool Proc() override;

    /** @brief Heartbeat aggregation and mode policy. */
    SupervisorImplementation supervisor_;
    /** @brief /navigation/heartbeat reader. */
    std::shared_ptr<autolink::Reader<lifecycle::Heartbeat>> navigation_reader_{nullptr};
    /** @brief /safety/heartbeat reader. */
    std::shared_ptr<autolink::Reader<lifecycle::Heartbeat>> safety_reader_{nullptr};
    /** @brief /behavior/heartbeat reader. */
    std::shared_ptr<autolink::Reader<lifecycle::Heartbeat>> behavior_reader_{nullptr};
    /** @brief /guardian/degradation writer. */
    std::shared_ptr<autolink::Writer<lifecycle::Degradation>> degradation_writer_{nullptr};
    /** @brief /guardian/heartbeat writer. */
    std::shared_ptr<autolink::Writer<lifecycle::Heartbeat>> heartbeat_writer_{nullptr};
    /** @brief Last published degradation mode. */
    lifecycle::DegradationMode mode_{lifecycle::DEGRADATION_MODE_UNSPECIFIED};
    /** @brief Monotonic message sequence. */
    std::uint64_t sequence_{0};
};

}  // namespace guardian
}  // namespace navigation
