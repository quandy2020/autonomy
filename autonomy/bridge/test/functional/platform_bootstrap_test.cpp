/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/tools/bootstrap.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace {

TEST(PlatformBootstrapTest, HealthAndReflectionFlags) {
    proto::GrpcOptions options;
    options.set_enable_health_check(true);
    options.set_enable_server_reflection(false);
    options.set_keepalive_time_ms(30000);

    common::async_grpc::Server::Builder builder;
    builder.SetServerAddress("127.0.0.1:0");
    builder.SetNumGrpcThreads(1);
    builder.SetNumEventThreads(1);

    PlatformApplyResult result;
    ApplyPlatform(builder, options, &result);

    EXPECT_TRUE(result.channel_args_applied);
    EXPECT_TRUE(result.credentials_applied);
    EXPECT_TRUE(result.health_enabled);
    EXPECT_TRUE(result.health_state.serving());
    EXPECT_FALSE(result.reflection_enabled);
    EXPECT_TRUE(result.interceptors.logging);
}

TEST(PlatformBootstrapTest, ReflectionRequestedWhenLibraryMissing) {
    proto::GrpcOptions options;
    options.set_enable_server_reflection(true);

    common::async_grpc::Server::Builder builder;
    builder.SetServerAddress("127.0.0.1:0");
    builder.SetNumGrpcThreads(1);
    builder.SetNumEventThreads(1);

    PlatformApplyResult result;
    ApplyPlatform(builder, options, &result);

    if (!result.reflection_library_available) {
        EXPECT_FALSE(result.reflection_enabled);
        GTEST_SKIP() << "grpc++_reflection not linked in this build";
    }
    EXPECT_TRUE(result.reflection_enabled);
}

TEST(PlatformBootstrapTest, MetadataInterceptorPlan) {
    proto::GrpcOptions options;
    options.set_enable_metadata_interceptor(true);
    options.set_require_robot_id_metadata(true);

    common::async_grpc::Server::Builder builder;
    builder.SetServerAddress("127.0.0.1:0");
    builder.SetNumGrpcThreads(1);
    builder.SetNumEventThreads(1);

    PlatformApplyResult result;
    ApplyPlatform(builder, options, &result);
    EXPECT_TRUE(result.interceptors.metadata);
}

}  // namespace
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
