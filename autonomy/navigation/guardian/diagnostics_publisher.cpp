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

#include "navigation/guardian/diagnostics_publisher.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace guardian
{

void DiagnosticsPublisher::Configure(double rate_hz)
{
    interval_ns_ = (std::isfinite(rate_hz) && rate_hz > 0.0)
                       ? static_cast<std::int64_t>(1e9 / rate_hz)
                       : 500000000LL;
    last_ns_.clear();
}

bool DiagnosticsPublisher::ShouldPublish(std::uint32_t code, std::int64_t now_ns)
{
    if (now_ns <= 0)
    {
        return false;
    }
    const auto iterator = last_ns_.find(code);
    if (iterator == last_ns_.end() || now_ns - iterator->second >= interval_ns_)
    {
        last_ns_[code] = now_ns;
        return true;
    }
    return false;
}

void DiagnosticsPublisher::Reset()
{
    last_ns_.clear();
}

}  // namespace guardian
}  // namespace navigation
