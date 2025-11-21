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

#include <chrono>
#include <future>
#include <thread>

#include "gtest/gtest.h"

#include "autolink/base/for_each.hpp"
#include "autolink/common/global_data.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autolink/scheduler/policy/classic_context.hpp"
#include "autolink/scheduler/processor.hpp"
#include "autolink/scheduler/scheduler_factory.hpp"
#include "autolink/time/time.hpp"

namespace autolink {
namespace scheduler {

using autolink::common::GlobalData;

void func() {}

TEST(SchedulerClassicTest, classic) {
    auto processor = std::make_shared<Processor>();
    auto ctx = std::make_shared<ClassicContext>();
    processor->BindContext(ctx);
    std::vector<std::future<void>> res;

    // test single routine
    auto future = std::async(std::launch::async, []() {
        FOR_EACH(i, 0, 20) {
            std::this_thread::sleep_for(std::chrono::milliseconds(i));
        }
        AINFO << "Finish task: single";
    });
    future.get();

    // test multiple routine
    FOR_EACH(i, 0, 20) {
        res.emplace_back(std::async(std::launch::async, [i]() {
            FOR_EACH(time, 0, 30) {
                std::this_thread::sleep_for(std::chrono::milliseconds(i));
            }
        }));
        AINFO << "Finish task: " << i;
    };
    for (auto& future : res) {
        future.wait_for(std::chrono::milliseconds(1000));
    }
    res.clear();
    ctx->Shutdown();
    processor->Stop();
}

TEST(SchedulerClassicTest, sched_classic) {
    // read example_sched_classic.conf
    GlobalData::Instance()->SetProcessGroup("example_sched_classic");
    auto sched1 = dynamic_cast<SchedulerClassic*>(scheduler::Instance());
    std::shared_ptr<CRoutine> cr = std::make_shared<CRoutine>(func);
    auto task_id = GlobalData::Instance()->RegisterTaskName("ABC");
    cr->set_id(task_id);
    cr->set_name("ABC");
    EXPECT_TRUE(sched1->DispatchTask(cr));
    // dispatch the same task
    EXPECT_FALSE(sched1->DispatchTask(cr));
    EXPECT_TRUE(sched1->RemoveTask("ABC"));

    std::shared_ptr<CRoutine> cr1 = std::make_shared<CRoutine>(func);
    cr1->set_id(GlobalData::Instance()->RegisterTaskName("xxxxxx"));
    cr1->set_name("xxxxxx");
    EXPECT_TRUE(sched1->DispatchTask(cr1));

    auto t = std::thread(func);
    sched1->SetInnerThreadAttr("shm", &t);
    if (t.joinable()) {
        t.join();
    }

    sched1->Shutdown();

    GlobalData::Instance()->SetProcessGroup("not_exist_sched");
    auto sched2 = dynamic_cast<SchedulerClassic*>(scheduler::Instance());
    std::shared_ptr<CRoutine> cr2 = std::make_shared<CRoutine>(func);
    cr2->set_id(GlobalData::Instance()->RegisterTaskName("sched2"));
    cr2->set_name("sched2");
    EXPECT_TRUE(sched2->DispatchTask(cr2));
    sched2->Shutdown();

    GlobalData::Instance()->SetProcessGroup("dreamview_sched");
    auto sched3 = dynamic_cast<SchedulerClassic*>(scheduler::Instance());
    std::shared_ptr<CRoutine> cr3 = std::make_shared<CRoutine>(func);
    cr3->set_id(GlobalData::Instance()->RegisterTaskName("sched3"));
    cr3->set_name("sched3");
    EXPECT_TRUE(sched3->DispatchTask(cr3));
    sched3->Shutdown();
}

}  // namespace scheduler
}  // namespace autolink