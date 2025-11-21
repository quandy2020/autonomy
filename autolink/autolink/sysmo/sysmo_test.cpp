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

#include "autolink/sysmo/sysmo.hpp"

#include <cstdlib>
#include <thread>

#include "gtest/gtest.h"

#include "autolink/common/environment.hpp"
#include "autolink/scheduler/scheduler_factory.hpp"

namespace autolink {

using autolink::common::GetEnv;

TEST(SysMoTest, cases) {
    auto sched = scheduler::Instance();
    setenv("sysmo_start", "1", 1);
    auto sysmo_start = GetEnv("sysmo_start");
    EXPECT_EQ(sysmo_start, "1");
    auto sysmo = SysMo::Instance();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    sysmo->Shutdown();
    sched->Shutdown();
}

}  // namespace autolink
