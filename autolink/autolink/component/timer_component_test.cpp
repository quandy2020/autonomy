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

#include "autolink/component/timer_component.hpp"

#include <memory>

#include "gtest/gtest.h"

#include "autolink/common/init.hpp"

namespace autolink {

static bool ret_proc = true;
static bool ret_init = true;

class Component_Timer : public TimerComponent
{
public:
    Component_Timer() {}
    bool Init() {
        return ret_init;
    }
    bool Proc() {
        return ret_proc;
    }
};

TEST(TimerComponent, timertest) {
    ret_proc = true;
    ret_init = true;
    autolink::Init("timer component test");
    autolink::proto::TimerComponentConfig compcfg;
    compcfg.set_name("driver");
    compcfg.set_interval(100);

    std::shared_ptr<Component_Timer> com = std::make_shared<Component_Timer>();
    EXPECT_TRUE(com->Initialize(compcfg));
    EXPECT_TRUE(com->Process());
}

TEST(TimerComponentFalse, timerfail) {
    ret_proc = false;
    ret_init = false;
    autolink::Init("timer component test");
    autolink::proto::TimerComponentConfig compcfg;
    compcfg.set_name("driver1");
    compcfg.set_interval(100);

    std::shared_ptr<Component_Timer> com = std::make_shared<Component_Timer>();
    EXPECT_FALSE(com->Initialize(compcfg));
    EXPECT_FALSE(com->Process());
}
}  // namespace autolink