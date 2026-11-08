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
 * @file heartbeat_monitor.hpp
 * @brief Per-module heartbeat freshness tracking.
 */

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "navigation/proto/lifecycle.pb.h"

namespace navigation
{
namespace guardian
{

/** @brief Freshness snapshot for one module. */
struct HeartbeatStatus
{
    /** @brief True when at least one heartbeat was seen. */
    bool received{false};
    /** @brief True when age <= timeout. */
    bool healthy{false};
    /** @brief True when age > fatal threshold. */
    bool fatal{false};
    /** @brief Last accepted sequence. */
    std::uint64_t last_sequence{0};
    /** @brief Age in nanoseconds. */
    std::int64_t age_ns{0};
};

/**
 * @brief Tracks the last heartbeat of each module.
 *
 * @par Threading
 * Owned by the guardian timer thread.
 */
class HeartbeatMonitor
{
public:
    /**
     * @brief Configure freshness windows.
     *
     * @param[in] timeout_ms Healthy window.
     * @param[in] fatal_ms Fatal window (>= timeout_ms).
     */
    void Configure(std::int64_t timeout_ms, std::int64_t fatal_ms);

    /** @brief Record a heartbeat; false when the sequence is stale. */
    bool Update(const lifecycle::Heartbeat& heartbeat, std::int64_t now_ns);

    /** @brief Check one module at a given time. */
    [[nodiscard]] HeartbeatStatus Check(const std::string& module,
                                        std::int64_t now_ns) const;

    /** @brief Clear all module entries. */
    void Reset();

private:
    /** @brief Per-module bookkeeping. */
    struct Entry
    {
        /** @brief Last accepted heartbeat time. */
        std::int64_t last_ns{0};
        /** @brief Last accepted sequence. */
        std::uint64_t last_sequence{0};
        /** @brief True after the first heartbeat. */
        bool received{false};
    };

    /** @brief Healthy window in nanoseconds. */
    std::int64_t timeout_ns_{200000000};
    /** @brief Fatal window in nanoseconds. */
    std::int64_t fatal_ns_{400000000};
    /** @brief Module -> bookkeeping. */
    std::unordered_map<std::string, Entry> entries_;
};

}  // namespace guardian
}  // namespace navigation
