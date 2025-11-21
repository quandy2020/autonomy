/**
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <limits>
#include <memory>
#include <mutex>

#include "autolink/base/macros.hpp"
#include "autolink/croutine/croutine.hpp"
namespace autolink {
namespace scheduler {

using autolink::croutine::CRoutine;

class ProcessorContext
{
public:
    virtual void Shutdown();
    virtual std::shared_ptr<CRoutine> NextRoutine() = 0;
    virtual void Wait() = 0;

protected:
    std::atomic<bool> stop_{false};
};

}  // namespace scheduler
}  // namespace autolink