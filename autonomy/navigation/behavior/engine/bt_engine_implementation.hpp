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
 * @file bt_engine_implementation.hpp
 * @brief Dependency-free fallback behavior engine.
 */

#pragma once

#include <cstdint>
#include <string_view>

#include "navigation/behavior/engine/bt_engine.hpp"

namespace navigation
{
namespace behavior
{

/**
 * @brief Minimal sequence/fallback engine used when BehaviorTree.CPP is absent.
 *
 * @details
 * Implements the decision skeleton of navigate_with_recovery: wait for a goal,
 * navigate, recover on module failure, and fail closed when recovery is
 * exhausted. Tree XML remains the authoritative description; this fallback
 * exists so Runner and unit tests do not depend on the optional BT library.
 */
class BtEngineImplementation final : public BtEngine
{
public:
    /** @brief Configure the component. */
    bool Configure(const config::BehaviorOptions& options,
                   planning::NavigationSession* session) override;

    /** @brief Advance one tick. */
    common::Status Tick(std::int64_t now_ns) override;

    /** @brief Reset local state. */
    void Reset() override;

    /** @brief Stable name. */
    [[nodiscard]] std::string_view Name() const override;

private:
    enum class State
    {
        kIdle = 0,
        kNavigate,
        kRecovery,
        kDone,
    };

    config::BehaviorOptions options_;
    planning::NavigationSession* session_{nullptr};
    State state_{State::kIdle};
    std::uint32_t recovery_attempts_{0};
    std::int64_t last_tick_ns_{0};
};

}  // namespace behavior
}  // namespace navigation
