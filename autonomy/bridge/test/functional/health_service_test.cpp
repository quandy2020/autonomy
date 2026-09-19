/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/tools/health/health_service.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace {

TEST(HealthServiceTest, ServingToggle) {
    HealthServiceState state;
    EXPECT_TRUE(state.serving());
    state.SetServing(false);
    EXPECT_FALSE(state.serving());
    state.SetServing(true);
    EXPECT_TRUE(state.serving());
}

TEST(HealthServiceTest, OverallServiceNameEmpty) {
    EXPECT_STREQ(HealthServiceState::OverallServiceName(), "");
}

}  // namespace
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
