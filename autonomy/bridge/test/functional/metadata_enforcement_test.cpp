/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/metadata/metadata_keys.hpp"
#include "autonomy/bridge/policy/metadata/metadata_validator.hpp"
#include "autonomy/bridge/tools/interceptors/server_interceptor_chain.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace {

TEST(MetadataEnforcementTest, RequireRobotIdRejects) {
    policy::MetadataValidator validator(/*require_robot_id=*/true);
    const auto missing = validator.Validate({});
    EXPECT_FALSE(missing.ok);
    EXPECT_EQ(missing.detail, "missing x-robot-id");

    const auto ok =
        validator.Validate({{policy::kMetadataKeyRobotId, "robot-a"}});
    EXPECT_TRUE(ok.ok);
}

TEST(MetadataEnforcementTest, ChainRegistersMetadataFactory) {
    proto::GrpcOptions options;
    options.set_enable_metadata_interceptor(true);
    options.set_require_robot_id_metadata(true);
    tools::interceptors::InterceptorChainPlan plan;
    auto factories =
        tools::interceptors::BuildInterceptorChain(options, &plan);
    EXPECT_TRUE(plan.metadata);
    EXPECT_GE(factories.size(), 2u);
}

}  // namespace
}  // namespace bridge
}  // namespace autonomy
