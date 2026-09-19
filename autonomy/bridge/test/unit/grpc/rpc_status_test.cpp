/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/rpc_status.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace {

TEST(RpcStatusTest, OkStatusDefaultsToOkCode) {
    const auto status = OkStatus();
    EXPECT_EQ(status.code(), ::automsgs::msgs::status_msgs::OK);
    EXPECT_TRUE(status.message().empty());
}

TEST(RpcStatusTest, OkStatusWithMessage) {
    const auto status = OkStatus("paused");
    EXPECT_EQ(status.code(), ::automsgs::msgs::status_msgs::OK);
    EXPECT_EQ(status.message(), "paused");
}

TEST(RpcStatusTest, ErrorStatusSetsCodeAndMessage) {
    const auto status = ErrorStatus(
        ::automsgs::msgs::status_msgs::INVALID_ARGUMENT, "bad arg");
    EXPECT_EQ(status.code(),
              ::automsgs::msgs::status_msgs::INVALID_ARGUMENT);
    EXPECT_EQ(status.message(), "bad arg");
}

}  // namespace
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
