/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/rate_limit/token_bucket.hpp"
#include "autonomy/bridge/tools/interceptors/server_interceptor_chain.hpp"
#include "gtest/gtest.h"

#include <thread>

namespace autonomy {
namespace bridge {
namespace {

TEST(RateLimitEnforcementTest, ExhaustThenRecover) {
    policy::TokenBucket bucket(/*qps=*/50.0, /*burst=*/2);
    EXPECT_TRUE(bucket.TryAcquire());
    EXPECT_TRUE(bucket.TryAcquire());
    EXPECT_FALSE(bucket.TryAcquire());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(bucket.TryAcquire());
}

TEST(RateLimitEnforcementTest, ChainRegistersWhenEnabled) {
    proto::GrpcOptions options;
    options.set_enable_rate_limit(true);
    options.set_rate_limit_qps(20.0);
    options.set_rate_limit_burst(5);
    tools::interceptors::InterceptorChainPlan plan;
    tools::interceptors::BuildInterceptorChain(options, &plan);
    EXPECT_TRUE(plan.rate_limit);
}

}  // namespace
}  // namespace bridge
}  // namespace autonomy
