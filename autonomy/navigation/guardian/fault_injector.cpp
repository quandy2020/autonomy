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

#include "navigation/guardian/fault_injector.hpp"

namespace navigation
{
namespace guardian
{

void FaultInjector::Enable(FaultKind kind)
{
    if (kind != FaultKind::kCount)
    {
        enabled_[static_cast<std::size_t>(kind)] = true;
    }
}

void FaultInjector::Disable(FaultKind kind)
{
    if (kind != FaultKind::kCount)
    {
        enabled_[static_cast<std::size_t>(kind)] = false;
    }
}

bool FaultInjector::IsEnabled(FaultKind kind) const
{
    if (kind == FaultKind::kCount)
    {
        return false;
    }
    return enabled_[static_cast<std::size_t>(kind)];
}

void FaultInjector::Reset()
{
    enabled_.fill(false);
}

}  // namespace guardian
}  // namespace navigation
