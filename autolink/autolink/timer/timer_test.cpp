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

#include "autolink/timer/timer.hpp"

#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "autolink/common/global_data.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/util.hpp"

namespace autolink {
namespace timer {

using autolink::Timer;
using autolink::TimerOption;

TEST(TimerTest, one_shot) {
    int count = 0;
    Timer timer(100, [&count] { count = 100; }, true);
    timer.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    EXPECT_EQ(0, count);
    // Here we need to consider the scheduling delay, up to 500ms, make sure the
    // unit test passes.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_EQ(100, count);
    timer.Stop();
}

TEST(TimerTest, cycle) {
    using TimerPtr = std::shared_ptr<Timer>;
    int count = 0;
    TimerPtr timers[1000];
    TimerOption opt;
    opt.oneshot = false;
    opt.callback = [=] { AINFO << count; };
    for (int i = 0; i < 1000; i++) {
        opt.period = i + 1;
        timers[i] = std::make_shared<Timer>();
        timers[i]->SetTimerOption(opt);
        timers[i]->Start();
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));
    for (int i = 0; i < 1000; i++) {
        timers[i]->Stop();
    }
}

TEST(TimerTest, start_stop) {
    int count = 0;
    Timer timer(2, [count] { AINFO << count; }, false);
    for (int i = 0; i < 100; i++) {
        timer.Start();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        timer.Stop();
    }
}

TEST(TimerTest, sim_mode) {
    auto count = 0;

    auto func = [count]() { AINFO << count; };

    TimerOption to{1000, func, false};

    {
        Timer t;
        t.SetTimerOption(to);
        common::GlobalData::Instance()->EnableSimulationMode();
        t.Start();
        common::GlobalData::Instance()->DisableSimulationMode();
        t.Start();
        t.Start();
    }
}

}  // namespace timer
}  // namespace autolink