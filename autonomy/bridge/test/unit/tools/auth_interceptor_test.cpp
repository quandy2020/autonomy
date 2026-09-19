/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/auth/token_authenticator.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace {

TEST(AuthInterceptorLogicTest, BearerRejectDetailStable) {
    proto::GrpcOptions options;
    options.set_auth_mode(proto::AUTH_MODE_BEARER_TOKEN);
    options.set_bearer_token("good");
    policy::TokenAuthenticator auth(options);
    const auto bad = auth.Authenticate("Bearer bad");
    EXPECT_FALSE(bad.ok);
    EXPECT_EQ(bad.detail, "missing or invalid bearer token");
    EXPECT_TRUE(auth.Authenticate("Bearer good").ok);
}

}  // namespace
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
