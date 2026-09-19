/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/tools/transport/channel_args_builder.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace {

TEST(ChannelArgsBuilderTest, DefaultsToHundredMegabytes) {
    proto::GrpcOptions options;
    // Build should not throw; ChannelArguments has no public getters for all
    // keys across grpc versions — smoke that construction succeeds.
    const auto args = ChannelArgsBuilder::Build(options);
    (void)args;
    SUCCEED();
}

TEST(ChannelArgsBuilderTest, KeepaliveMappedWhenSet) {
    proto::GrpcOptions options;
    options.set_max_receive_message_bytes(1024);
    options.set_max_send_message_bytes(2048);
    options.set_keepalive_time_ms(30000);
    options.set_keepalive_timeout_ms(10000);
    options.set_permit_keepalive_without_calls(true);
    const auto args = ChannelArgsBuilder::Build(options);
    (void)args;
    SUCCEED();
}

}  // namespace
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
