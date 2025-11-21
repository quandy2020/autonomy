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

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "autolink/proto/scheduler_conf.pb.h"

#include "autolink/croutine/croutine.hpp"
#include "autolink/scheduler/processor_context.hpp"

namespace autolink {
namespace scheduler {

using croutine::CRoutine;

struct Snapshot {
    std::atomic<uint64_t> execute_start_time = {0};
    std::atomic<pid_t> processor_id = {0};
    std::string routine_name;
};

class Processor
{
public:
    Processor();
    virtual ~Processor();

    void Run();
    void Stop();
    void BindContext(const std::shared_ptr<ProcessorContext>& context);
    std::thread* Thread() {
        return &thread_;
    }
    std::atomic<pid_t>& Tid();

    std::shared_ptr<Snapshot> ProcSnapshot() {
        return snap_shot_;
    }

private:
    std::shared_ptr<ProcessorContext> context_;

    std::condition_variable cv_ctx_;
    std::once_flag thread_flag_;
    std::mutex mtx_ctx_;
    std::thread thread_;

    std::atomic<pid_t> tid_{-1};
    std::atomic<bool> running_{false};

    std::shared_ptr<Snapshot> snap_shot_ = std::make_shared<Snapshot>();
};

}  // namespace scheduler
}  // namespace autolink