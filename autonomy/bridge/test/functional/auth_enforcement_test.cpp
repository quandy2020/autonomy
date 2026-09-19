/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/auth/token_authenticator.hpp"
#include "autonomy/bridge/tools/interceptors/server_interceptor_chain.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace {

TEST(AuthEnforcementTest, BearerWrongRejectedCorrectAccepted) {
    proto::GrpcOptions options;
    options.set_auth_mode(proto::AUTH_MODE_BEARER_TOKEN);
    options.set_bearer_token("s3cret");
    policy::TokenAuthenticator auth(options);
    EXPECT_FALSE(auth.Authenticate("Bearer wrong").ok);
    EXPECT_TRUE(auth.Authenticate("Bearer s3cret").ok);
}

TEST(AuthEnforcementTest, ChainRegistersAuthFactory) {
    proto::GrpcOptions options;
    options.set_auth_mode(proto::AUTH_MODE_BEARER_TOKEN);
    options.set_bearer_token("t");
    tools::interceptors::InterceptorChainPlan plan;
    tools::interceptors::BuildInterceptorChain(options, &plan);
    EXPECT_TRUE(plan.auth);
}

}  // namespace
}  // namespace bridge
}  // namespace autonomy
