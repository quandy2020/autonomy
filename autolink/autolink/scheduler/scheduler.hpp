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

#include <unistd.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "autolink/proto/choreography_conf.pb.h"

#include "autolink/base/atomic_hash_map.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/macros.hpp"
#include "autolink/common/types.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autolink/croutine/routine_factory.hpp"
#include "autolink/scheduler/common/mutex_wrapper.hpp"
#include "autolink/scheduler/processor_context.hpp"

namespace autolink {
namespace scheduler {

using autolink::base::AtomicHashMap;
using autolink::base::AtomicRWLock;
using autolink::base::ReadLockGuard;
using autolink::croutine::CRoutine;
using autolink::croutine::RoutineFactory;
using autolink::data::DataVisitorBase;
using autolink::proto::InnerThread;

class Processor;
class ProcessorContext;

class Scheduler
{
public:
    virtual ~Scheduler() {}
    static Scheduler* Instance();

    bool CreateTask(const RoutineFactory& factory, const std::string& name);
    bool CreateTask(std::function<void()>&& func, const std::string& name,
                    std::shared_ptr<DataVisitorBase> visitor = nullptr);
    bool NotifyTask(uint64_t crid);

    void Shutdown();
    uint32_t TaskPoolSize() {
        return task_pool_size_;
    }

    virtual bool RemoveTask(const std::string& name) = 0;

    void ProcessLevelResourceControl();
    void SetInnerThreadAttr(const std::string& name, std::thread* thr);

    virtual bool DispatchTask(const std::shared_ptr<CRoutine>&) = 0;
    virtual bool NotifyProcessor(uint64_t crid) = 0;
    virtual bool RemoveCRoutine(uint64_t crid) = 0;

    void CheckSchedStatus();

    void SetInnerThreadConfs(
        const std::unordered_map<std::string, InnerThread>& confs) {
        inner_thr_confs_ = confs;
    }

protected:
    Scheduler() : stop_(false) {}

    AtomicRWLock id_cr_lock_;
    AtomicHashMap<uint64_t, MutexWrapper*> id_map_mutex_;
    std::mutex cr_wl_mtx_;

    std::unordered_map<uint64_t, std::shared_ptr<CRoutine>> id_cr_;
    std::vector<std::shared_ptr<ProcessorContext>> pctxs_;
    std::vector<std::shared_ptr<Processor>> processors_;

    std::unordered_map<std::string, InnerThread> inner_thr_confs_;

    std::string process_level_cpuset_;
    uint32_t proc_num_ = 0;
    uint32_t task_pool_size_ = 0;
    std::atomic<bool> stop_;
};

}  // namespace scheduler
}  // namespace autolink