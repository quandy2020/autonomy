/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/reliability/deadline_policy.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace policy {
namespace {

TEST(DeadlinePolicyTest, MaxDeadlineNeverExpired) {
    const auto now = std::chrono::system_clock::now();
    EXPECT_FALSE(DeadlinePolicy::IsExpired(
        now, std::chrono::system_clock::time_point::max()));
}

TEST(DeadlinePolicyTest, ZeroEpochNeverExpired) {
    const auto now = std::chrono::system_clock::now();
    EXPECT_FALSE(DeadlinePolicy::IsExpired(
        now, std::chrono::system_clock::time_point{}));
}

TEST(DeadlinePolicyTest, PastDeadlineExpired) {
    const auto now = std::chrono::system_clock::now();
    const auto past = now - std::chrono::seconds(1);
    EXPECT_TRUE(DeadlinePolicy::IsExpired(now, past));
}

TEST(DeadlinePolicyTest, FutureDeadlineNotExpired) {
    const auto now = std::chrono::system_clock::now();
    const auto future = now + std::chrono::seconds(60);
    EXPECT_FALSE(DeadlinePolicy::IsExpired(now, future));
}

}  // namespace
}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
