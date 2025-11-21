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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autolink/proto/choreography_conf.pb.h"

#include "autolink/croutine/croutine.hpp"
#include "autolink/scheduler/scheduler.hpp"

namespace autolink {
namespace scheduler {

using autolink::croutine::CRoutine;
using autolink::proto::ChoreographyTask;

class SchedulerChoreography : public Scheduler
{
public:
    bool RemoveCRoutine(uint64_t crid) override;
    bool RemoveTask(const std::string& name) override;
    bool DispatchTask(const std::shared_ptr<CRoutine>&) override;

private:
    friend Scheduler* Instance();
    SchedulerChoreography();

    void CreateProcessor();
    bool NotifyProcessor(uint64_t crid) override;

    std::unordered_map<std::string, ChoreographyTask> cr_confs_;

    int32_t choreography_processor_prio_;
    int32_t pool_processor_prio_;

    std::string choreography_affinity_;
    std::string pool_affinity_;

    std::string choreography_processor_policy_;
    std::string pool_processor_policy_;

    std::vector<int> choreography_cpuset_;
    std::vector<int> pool_cpuset_;
};

}  // namespace scheduler
}  // namespace autolink