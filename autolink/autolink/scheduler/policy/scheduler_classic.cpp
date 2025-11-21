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

#include "autolink/scheduler/policy/scheduler_classic.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "autolink/common/environment.hpp"
#include "autolink/common/file.hpp"
#include "autolink/scheduler/policy/classic_context.hpp"
#include "autolink/scheduler/processor.hpp"

namespace autolink {
namespace scheduler {

using autolink::base::ReadLockGuard;
using autolink::base::WriteLockGuard;
using autolink::common::GetAbsolutePath;
using autolink::common::GetProtoFromFile;
using autolink::common::GlobalData;
using autolink::common::PathExists;
using autolink::common::WorkRoot;
using autolink::croutine::RoutineState;

SchedulerClassic::SchedulerClassic() {
    std::string conf("conf/");
    conf.append(GlobalData::Instance()->ProcessGroup()).append(".conf");
    auto cfg_file = GetAbsolutePath(WorkRoot(), conf);

    autolink::proto::AutoLinkConfig cfg;
    if (PathExists(cfg_file) && GetProtoFromFile(cfg_file, &cfg)) {
        for (auto& thr : cfg.scheduler_conf().threads()) {
            inner_thr_confs_[thr.name()] = thr;
        }

        // proto3: 空字符串代表未配置
        process_level_cpuset_ = cfg.scheduler_conf().process_level_cpuset();
        if (!process_level_cpuset_.empty()) {
            ProcessLevelResourceControl();
        }

        classic_conf_ = cfg.scheduler_conf().classic_conf();
        for (auto& group : classic_conf_.groups()) {
            auto& group_name = group.name();
            for (auto task : group.tasks()) {
                task.set_group_name(group_name);
                cr_confs_[task.name()] = task;
            }
        }
    } else {
        // if not set default_proc_num in scheduler conf, use default value
        uint32_t proc_num = 2;
        auto& global_conf = GlobalData::Instance()->Config();
        if (global_conf.has_scheduler_conf()) {
            auto dpn = global_conf.scheduler_conf().default_proc_num();
            if (dpn > 0) {
                proc_num = dpn;
            }
        }
        task_pool_size_ = proc_num;

        auto sched_group = classic_conf_.add_groups();
        sched_group->set_name(DEFAULT_GROUP_NAME);
        sched_group->set_processor_num(proc_num);
    }

    CreateProcessor();
}

void SchedulerClassic::CreateProcessor() {
    for (auto& group : classic_conf_.groups()) {
        auto& group_name = group.name();
        auto proc_num = group.processor_num();
        if (task_pool_size_ == 0) {
            task_pool_size_ = proc_num;
        }

        // TODO: Use these variables when implementing CPU affinity and
        // processor policy
        [[maybe_unused]] auto& affinity = group.affinity();
        [[maybe_unused]] auto& processor_policy = group.processor_policy();
        [[maybe_unused]] auto processor_prio = group.processor_prio();
        std::vector<int> cpuset;  // TODO: parse cpuset when utility available

        for (uint32_t i = 0; i < proc_num; i++) {
            auto ctx = std::make_shared<ClassicContext>(group_name);
            pctxs_.emplace_back(ctx);

            auto proc = std::make_shared<Processor>();
            proc->BindContext(ctx);
            // TODO: apply affinity/policy when utilities are available
            processors_.emplace_back(proc);
        }
    }
}

bool SchedulerClassic::DispatchTask(const std::shared_ptr<CRoutine>& cr) {
    // we use multi-key mutex to prevent race condition
    // when del && add cr with same crid
    MutexWrapper* wrapper = nullptr;
    if (!id_map_mutex_.Get(cr->id(), &wrapper)) {
        {
            std::lock_guard<std::mutex> wl_lg(cr_wl_mtx_);
            if (!id_map_mutex_.Get(cr->id(), &wrapper)) {
                wrapper = new MutexWrapper();
                id_map_mutex_.Set(cr->id(), wrapper);
            }
        }
    }
    std::lock_guard<std::mutex> lg(wrapper->Mutex());

    {
        WriteLockGuard<AtomicRWLock> lk(id_cr_lock_);
        if (id_cr_.find(cr->id()) != id_cr_.end()) {
            return false;
        }
        id_cr_[cr->id()] = cr;
    }

    if (cr_confs_.find(cr->name()) != cr_confs_.end()) {
        ClassicTask task = cr_confs_[cr->name()];
        cr->set_priority(task.prio());
        cr->set_group_name(task.group_name());
    } else {
        // croutine that not exist in conf
        cr->set_group_name(classic_conf_.groups(0).name());
    }

    if (cr->priority() >= MAX_PRIO) {
        AWARN << cr->name() << " prio is greater than MAX_PRIO[ << " << MAX_PRIO
              << "].";
        cr->set_priority(MAX_PRIO - 1);
    }

    // Enqueue task.
    {
        WriteLockGuard<AtomicRWLock> lk(
            ClassicContext::rq_locks_[cr->group_name()].at(cr->priority()));
        ClassicContext::cr_group_[cr->group_name()]
            .at(cr->priority())
            .emplace_back(cr);
    }

    ClassicContext::Notify(cr->group_name());
    return true;
}

bool SchedulerClassic::NotifyProcessor(uint64_t crid) {
    if (autolink_unlikely(stop_)) {
        return true;
    }

    {
        ReadLockGuard<AtomicRWLock> lk(id_cr_lock_);
        if (id_cr_.find(crid) != id_cr_.end()) {
            auto cr = id_cr_[crid];
            if (cr->state() == RoutineState::DATA_WAIT ||
                cr->state() == RoutineState::IO_WAIT) {
                cr->SetUpdateFlag();
            }

            ClassicContext::Notify(cr->group_name());
            return true;
        }
    }
    return false;
}

bool SchedulerClassic::RemoveTask(const std::string& name) {
    if (autolink_unlikely(stop_)) {
        return true;
    }

    auto crid = GlobalData::GenerateHashId(name);
    return RemoveCRoutine(crid);
}

bool SchedulerClassic::RemoveCRoutine(uint64_t crid) {
    // we use multi-key mutex to prevent race condition
    // when del && add cr with same crid
    MutexWrapper* wrapper = nullptr;
    if (!id_map_mutex_.Get(crid, &wrapper)) {
        {
            std::lock_guard<std::mutex> wl_lg(cr_wl_mtx_);
            if (!id_map_mutex_.Get(crid, &wrapper)) {
                wrapper = new MutexWrapper();
                id_map_mutex_.Set(crid, wrapper);
            }
        }
    }
    std::lock_guard<std::mutex> lg(wrapper->Mutex());

    std::shared_ptr<CRoutine> cr = nullptr;
    {
        WriteLockGuard<AtomicRWLock> lk(id_cr_lock_);
        if (id_cr_.find(crid) != id_cr_.end()) {
            cr = id_cr_[crid];
            id_cr_[crid]->Stop();
            id_cr_.erase(crid);
        } else {
            return false;
        }
    }
    return ClassicContext::RemoveCRoutine(cr);
}

}  // namespace scheduler
}  // namespace autolink