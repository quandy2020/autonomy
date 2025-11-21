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

#include "autolink/scheduler/policy/choreography_context.hpp"

#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autolink/common/types.hpp"

namespace autolink {
namespace scheduler {

using autolink::base::ReadLockGuard;
using autolink::base::WriteLockGuard;

using autolink::croutine::RoutineState;

std::shared_ptr<CRoutine> ChoreographyContext::NextRoutine() {
    if (autolink_unlikely(stop_.load())) {
        return nullptr;
    }

    ReadLockGuard<AtomicRWLock> lock(rq_lk_);
    for (auto it : cr_queue_) {
        auto cr = it.second;
        if (!cr->Acquire()) {
            continue;
        }

        if (cr->UpdateState() == RoutineState::READY) {
            return cr;
        }
        cr->Release();
    }
    return nullptr;
}

bool ChoreographyContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
    WriteLockGuard<AtomicRWLock> lock(rq_lk_);
    cr_queue_.emplace(cr->priority(), cr);
    return true;
}

void ChoreographyContext::Notify() {
    mtx_wq_.lock();
    notify++;
    mtx_wq_.unlock();
    cv_wq_.notify_one();
}

void ChoreographyContext::Wait() {
    std::unique_lock<std::mutex> lk(mtx_wq_);
    cv_wq_.wait_for(lk, std::chrono::milliseconds(1000),
                    [&]() { return notify > 0; });
    if (notify > 0) {
        notify--;
    }
}

void ChoreographyContext::Shutdown() {
    stop_.store(true);
    mtx_wq_.lock();
    notify = std::numeric_limits<unsigned char>::max();
    mtx_wq_.unlock();
    cv_wq_.notify_all();
}

bool ChoreographyContext::RemoveCRoutine(uint64_t crid) {
    WriteLockGuard<AtomicRWLock> lock(rq_lk_);
    for (auto it = cr_queue_.begin(); it != cr_queue_.end();) {
        auto cr = it->second;
        if (cr->id() == crid) {
            cr->Stop();
            while (!cr->Acquire()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                AINFO_EVERY(1000)
                    << "waiting for task " << cr->name() << " completion";
            }
            it = cr_queue_.erase(it);
            cr->Release();
            return true;
        }
        ++it;
    }
    return false;
}
}  // namespace scheduler
}  // namespace autolink