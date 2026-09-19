/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/tools/interceptors/server_interceptor_chain.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {
namespace {

TEST(ServerInterceptorChainTest, AlwaysIncludesLogging) {
    proto::GrpcOptions options;
    InterceptorChainPlan plan;
    auto factories = BuildInterceptorChain(options, &plan);
    EXPECT_TRUE(plan.logging);
    EXPECT_FALSE(factories.empty());
}

TEST(ServerInterceptorChainTest, AuthOnlyWhenBearer) {
    proto::GrpcOptions options;
    options.set_auth_mode(proto::AUTH_MODE_BEARER_TOKEN);
    options.set_bearer_token("t");
    InterceptorChainPlan plan;
    BuildInterceptorChain(options, &plan);
    EXPECT_TRUE(plan.auth);
}

TEST(ServerInterceptorChainTest, MetadataWhenEnabled) {
    proto::GrpcOptions options;
    options.set_enable_metadata_interceptor(true);
    InterceptorChainPlan plan;
    BuildInterceptorChain(options, &plan);
    EXPECT_TRUE(plan.metadata);
}

TEST(ServerInterceptorChainTest, RateLimitWhenEnabled) {
    proto::GrpcOptions options;
    options.set_enable_rate_limit(true);
    options.set_rate_limit_qps(10.0);
    InterceptorChainPlan plan;
    BuildInterceptorChain(options, &plan);
    EXPECT_TRUE(plan.rate_limit);
}

TEST(ServerInterceptorChainTest, OrderFlagsIndependent) {
    proto::GrpcOptions options;
    options.set_auth_mode(proto::AUTH_MODE_NONE);
    options.set_enable_metadata_interceptor(false);
    options.set_enable_rate_limit(false);
    options.set_enable_opentelemetry(false);
    InterceptorChainPlan plan;
    auto factories = BuildInterceptorChain(options, &plan);
    EXPECT_EQ(factories.size(), 1u);  // logging only
    EXPECT_TRUE(plan.logging);
    EXPECT_FALSE(plan.auth);
    EXPECT_FALSE(plan.metadata);
    EXPECT_FALSE(plan.rate_limit);
    EXPECT_FALSE(plan.otel);
}

}  // namespace
}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
