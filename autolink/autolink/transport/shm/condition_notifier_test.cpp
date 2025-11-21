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

#include "autolink/transport/shm/condition_notifier.hpp"

#include "gtest/gtest.h"

namespace autolink {
namespace transport {

TEST(ConditionNotifierTest, constructor) {
    auto notifier = ConditionNotifier::Instance();
    EXPECT_NE(notifier, nullptr);
}

TEST(ConditionNotifierTest, notify_listen) {
    auto notifier = ConditionNotifier::Instance();
    ReadableInfo readable_info;
    while (notifier->Listen(100, &readable_info)) {
    }
    EXPECT_FALSE(notifier->Listen(100, &readable_info));
    EXPECT_TRUE(notifier->Notify(readable_info));
    EXPECT_TRUE(notifier->Listen(100, &readable_info));
    EXPECT_FALSE(notifier->Listen(100, &readable_info));
    EXPECT_TRUE(notifier->Notify(readable_info));
    EXPECT_TRUE(notifier->Notify(readable_info));
    EXPECT_TRUE(notifier->Listen(100, &readable_info));
    EXPECT_TRUE(notifier->Listen(100, &readable_info));
    EXPECT_FALSE(notifier->Listen(100, &readable_info));
}

TEST(ConditionNotifierTest, shutdown) {
    auto notifier = ConditionNotifier::Instance();
    notifier->Shutdown();
    ReadableInfo readable_info;
    EXPECT_FALSE(notifier->Notify(readable_info));
    EXPECT_FALSE(notifier->Listen(100, &readable_info));
}

}  // namespace transport
}  // namespace autolink