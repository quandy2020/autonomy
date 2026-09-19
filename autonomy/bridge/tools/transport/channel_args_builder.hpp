/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file channel_args_builder.hpp
 * @brief Build grpc::ChannelArguments from GrpcOptions.
 */

#pragma once

#include "autonomy/bridge/proto/grpc_options.pb.h"
#include "autonomy/common/macros.hpp"
#include "grpcpp/support/channel_arguments.h"

namespace autonomy {
namespace bridge {
namespace tools {

/**
 * @brief Maps GrpcOptions transport fields into ChannelArguments.
 */
class ChannelArgsBuilder
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(ChannelArgsBuilder)

    static constexpr int kDefaultMaxMessageBytes = 100 * 1024 * 1024;

    /**
     * @brief Build channel args (message size + optional keepalive).
     */
    static ::grpc::ChannelArguments Build(const proto::GrpcOptions& options);
};

}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
