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

#include "autolink/scheduler/processor.hpp"

#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>

#include <chrono>

#include "autolink/common/global_data.hpp"
#include "autolink/common/log.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autolink/time/time.hpp"

namespace autolink {
namespace scheduler {

using autolink::common::GlobalData;

Processor::Processor() {
    running_.store(true);
}

Processor::~Processor() {
    Stop();
}

void Processor::Run() {
    tid_.store(static_cast<int>(syscall(SYS_gettid)));
    AINFO << "processor_tid: " << tid_;
    snap_shot_->processor_id.store(tid_);

    while (autolink_likely(running_.load())) {
        if (autolink_likely(context_ != nullptr)) {
            auto croutine = context_->NextRoutine();
            if (croutine) {
                snap_shot_->execute_start_time.store(
                    autolink::Time::Now().ToNanosecond());
                snap_shot_->routine_name = croutine->name();
                croutine->Resume();
                croutine->Release();
            } else {
                snap_shot_->execute_start_time.store(0);
                context_->Wait();
            }
        } else {
            std::unique_lock<std::mutex> lk(mtx_ctx_);
            cv_ctx_.wait_for(lk, std::chrono::milliseconds(10));
        }
    }
}

void Processor::Stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (context_) {
        context_->Shutdown();
    }

    cv_ctx_.notify_one();
    if (thread_.joinable()) {
        thread_.join();
    }
}

void Processor::BindContext(const std::shared_ptr<ProcessorContext>& context) {
    context_ = context;
    std::call_once(thread_flag_,
                   [this]() { thread_ = std::thread(&Processor::Run, this); });
}

std::atomic<pid_t>& Processor::Tid() {
    while (tid_.load() == -1) {
        cpu_relax();
    }
    return tid_;
}

}  // namespace scheduler
}  // namespace autolink