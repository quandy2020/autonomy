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

#include "autolink/croutine/croutine.hpp"

#include <algorithm>
#include <utility>

#include "autolink/base/concurrent_object_pool.hpp"
#include "autolink/common/global_data.hpp"
#include "autolink/common/log.hpp"
#include "autolink/croutine/detail/routine_context.hpp"

namespace autolink {
namespace croutine {

thread_local CRoutine* CRoutine::current_routine_ = nullptr;
thread_local char* CRoutine::main_stack_ = nullptr;

namespace {
std::shared_ptr<base::CCObjectPool<RoutineContext>> context_pool = nullptr;
std::once_flag pool_init_flag;

void CRoutineEntry(void* arg) {
    CRoutine* r = static_cast<CRoutine*>(arg);
    r->Run();
    CRoutine::Yield(RoutineState::FINISHED);
}
}  // namespace

CRoutine::CRoutine(const std::function<void()>& func) : func_(func) {
    std::call_once(pool_init_flag, [&]() {
        uint32_t routine_num = common::GlobalData::Instance()->ComponentNums();
        auto& global_conf = common::GlobalData::Instance()->Config();
        if (global_conf.has_scheduler_conf()) {
            // proto3: 无 has_* 标志的标量字段。以 >0 作为是否配置的条件。
            auto rn = global_conf.scheduler_conf().routine_num();
            if (rn > 0) {
                routine_num = std::max(routine_num, rn);
            }
        }
        context_pool.reset(new base::CCObjectPool<RoutineContext>(routine_num));
    });

    context_ = context_pool->GetObject();
    if (context_ == nullptr) {
        AWARN << "Maximum routine context number exceeded! Please check "
                 "[routine_num] in config file.";
        context_.reset(new RoutineContext());
    }

    MakeContext(CRoutineEntry, this, context_.get());
    state_ = RoutineState::READY;
    updated_.test_and_set(std::memory_order_release);
}

CRoutine::~CRoutine() {
    context_ = nullptr;
}

RoutineState CRoutine::Resume() {
    if (autolink_unlikely(force_stop_)) {
        state_ = RoutineState::FINISHED;
        return state_;
    }

    if (autolink_unlikely(state_ != RoutineState::READY)) {
        AERROR << "Invalid Routine State!";
        return state_;
    }

    current_routine_ = this;
    SwapContext(GetMainStack(), GetStack());
    current_routine_ = nullptr;
    return state_;
}

void CRoutine::Stop() {
    force_stop_ = true;
}

}  // namespace croutine
}  // namespace autolink