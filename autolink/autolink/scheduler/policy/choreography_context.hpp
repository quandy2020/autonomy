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

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autolink/scheduler/processor_context.hpp"

namespace autolink {
namespace scheduler {

using autolink::base::AtomicRWLock;
using croutine::CRoutine;

class ChoreographyContext : public ProcessorContext
{
public:
    virtual ~ChoreographyContext() = default;
    bool RemoveCRoutine(uint64_t crid);
    std::shared_ptr<CRoutine> NextRoutine() override;

    bool Enqueue(const std::shared_ptr<CRoutine>&);
    void Notify();
    void Wait() override;
    void Shutdown() override;

private:
    std::mutex mtx_wq_;
    std::condition_variable cv_wq_;
    int notify = 0;

    AtomicRWLock rq_lk_;
    std::multimap<uint32_t, std::shared_ptr<CRoutine>, std::greater<uint32_t>>
        cr_queue_;
};

}  // namespace scheduler
}  // namespace autolink