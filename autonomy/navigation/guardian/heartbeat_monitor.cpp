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

#include "navigation/guardian/heartbeat_monitor.hpp"

#include <algorithm>

namespace navigation
{
namespace guardian
{

void HeartbeatMonitor::Configure(std::int64_t timeout_ms, std::int64_t fatal_ms)
{
    timeout_ns_ = std::max<std::int64_t>(1, timeout_ms) * 1000000LL;
    fatal_ns_ = std::max(timeout_ns_, fatal_ms * 1000000LL);
}

bool HeartbeatMonitor::Update(const lifecycle::Heartbeat& heartbeat, std::int64_t now_ns)
{
    if (heartbeat.module().empty() || heartbeat.stamp_ns() <= 0 || now_ns <= 0)
    {
        return false;
    }
    auto& entry = entries_[heartbeat.module()];
    if (entry.received && heartbeat.sequence() <= entry.last_sequence)
    {
        return false;
    }
    entry.received = true;
    entry.last_sequence = heartbeat.sequence();
    entry.last_ns = now_ns;
    return true;
}

HeartbeatStatus HeartbeatMonitor::Check(const std::string& module,
                                        std::int64_t now_ns) const
{
    HeartbeatStatus status;
    const auto iterator = entries_.find(module);
    if (iterator == entries_.end() || !iterator->second.received)
    {
        status.fatal = true;
        return status;
    }
    status.received = true;
    status.last_sequence = iterator->second.last_sequence;
    status.age_ns = std::max<std::int64_t>(0, now_ns - iterator->second.last_ns);
    status.healthy = status.age_ns <= timeout_ns_;
    status.fatal = status.age_ns > fatal_ns_;
    return status;
}

void HeartbeatMonitor::Reset()
{
    entries_.clear();
}

}  // namespace guardian
}  // namespace navigation
