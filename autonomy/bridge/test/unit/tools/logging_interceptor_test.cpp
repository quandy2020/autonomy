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

TEST(LoggingInterceptorTest, ChainAlwaysRegistersLogging) {
    proto::GrpcOptions options;
    InterceptorChainPlan plan;
    BuildInterceptorChain(options, &plan);
    EXPECT_TRUE(plan.logging);
}

}  // namespace
}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
