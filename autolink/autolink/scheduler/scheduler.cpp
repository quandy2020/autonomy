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

#include "autolink/scheduler/scheduler.hpp"

#include <sched.h>

#include <utility>

#ifdef __APPLE__
// macOS doesn't support cpu_set_t
#define CPU_ZERO(set) \
    do {              \
        (void)(set);  \
    } while (0)
#define CPU_SET(cpu, set) \
    do {                  \
        (void)(cpu);      \
        (void)(set);      \
    } while (0)
typedef int cpu_set_t;
#endif

#include "autolink/common/environment.hpp"
#include "autolink/common/file.hpp"
#include "autolink/common/global_data.hpp"
#include "autolink/common/util.hpp"
#include "autolink/data/data_visitor.hpp"
#include "autolink/scheduler/processor.hpp"
#include "autolink/scheduler/processor_context.hpp"

namespace autolink {
namespace scheduler {

using autolink::common::GlobalData;

bool Scheduler::CreateTask(const RoutineFactory& factory,
                           const std::string& name) {
    return CreateTask(factory.create_routine(), name, factory.GetDataVisitor());
}

bool Scheduler::CreateTask(std::function<void()>&& func,
                           const std::string& name,
                           std::shared_ptr<DataVisitorBase> visitor) {
    if (autolink_unlikely(stop_.load())) {
        ADEBUG << "scheduler is stoped, cannot create task!";
        return false;
    }

    auto task_id = GlobalData::RegisterTaskName(name);

    auto cr = std::make_shared<CRoutine>(func);
    cr->set_id(task_id);
    cr->set_name(name);
    AINFO << "create croutine: " << name;

    if (!DispatchTask(cr)) {
        return false;
    }

    if (visitor != nullptr) {
        visitor->RegisterNotifyCallback([this, task_id]() {
            if (autolink_unlikely(stop_.load())) {
                return;
            }
            this->NotifyProcessor(task_id);
        });
    }
    return true;
}

bool Scheduler::NotifyTask(uint64_t crid) {
    if (autolink_unlikely(stop_.load())) {
        return true;
    }
    return NotifyProcessor(crid);
}

void Scheduler::ProcessLevelResourceControl() {
#ifdef __APPLE__
    // macOS doesn't support pthread_setaffinity_np
    // CPU affinity is not available on macOS
    ADEBUG << "CPU affinity not supported on macOS";
#else
    std::vector<int> cpus;  // TODO: parse cpuset string when implemented
    cpu_set_t set;
    CPU_ZERO(&set);
    for (const auto cpu : cpus) {
        CPU_SET(cpu, &set);
    }
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
#endif
}

void Scheduler::SetInnerThreadAttr(const std::string& name, std::thread* thr) {
    if (thr != nullptr &&
        inner_thr_confs_.find(name) != inner_thr_confs_.end()) {
        auto th_conf = inner_thr_confs_[name];
        auto cpuset = th_conf.cpuset();

        std::vector<int> cpus;  // TODO: parse cpuset when implemented
        (void)
            cpus;  // suppress unused for now
                   // TODO: Implement SetSchedAffinity/SetSchedPolicy if needed
    }
}

void Scheduler::CheckSchedStatus() {
    std::string snap_info;
    auto now = Time::Now().ToNanosecond();
    for (auto processor : processors_) {
        auto snap = processor->ProcSnapshot();
        if (snap->execute_start_time.load()) {
            auto execute_time =
                (now - snap->execute_start_time.load()) / 1000000;
            snap_info.append(std::to_string(snap->processor_id.load()))
                .append(":")
                .append(snap->routine_name)
                .append(":")
                .append(std::to_string(execute_time));
        } else {
            snap_info.append(std::to_string(snap->processor_id.load()))
                .append(":idle");
        }
        snap_info.append(", ");
    }
    snap_info.append("timestamp: ").append(std::to_string(now));
    AINFO << snap_info;
    snap_info.clear();
}

void Scheduler::Shutdown() {
    if (autolink_unlikely(stop_.exchange(true))) {
        return;
    }

    for (auto& ctx : pctxs_) {
        ctx->Shutdown();
    }

    std::vector<uint64_t> cr_list;
    {
        ReadLockGuard<AtomicRWLock> lk(id_cr_lock_);
        for (auto& cr : id_cr_) {
            cr_list.emplace_back(cr.second->id());
        }
    }

    for (auto& id : cr_list) {
        RemoveCRoutine(id);
    }

    for (auto& processor : processors_) {
        processor->Stop();
    }

    processors_.clear();
    pctxs_.clear();
}
}  // namespace scheduler
}  // namespace autolink