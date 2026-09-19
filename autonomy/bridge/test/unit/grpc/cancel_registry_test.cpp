/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/cancel_registry.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace {

TEST(CancelRegistryTest, NamedFactoryHookRuns) {
    CancelRegistry registry;
    int hits = 0;
    ASSERT_TRUE(registry.RegisterCancel("a", [&] { ++hits; }));
    ASSERT_TRUE(registry.RegisterCancel("b", [&] { hits += 10; }));
    EXPECT_FALSE(registry.RegisterCancel("a", [&] {}));  // duplicate id
    EXPECT_TRUE(registry.Contains("a"));
    registry.CancelAll();
    EXPECT_EQ(hits, 11);
}

}  // namespace
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
