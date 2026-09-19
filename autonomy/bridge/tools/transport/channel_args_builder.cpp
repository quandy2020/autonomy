/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/tools/transport/channel_args_builder.hpp"

#include <grpc/grpc.h>

namespace autonomy {
namespace bridge {
namespace tools {

::grpc::ChannelArguments ChannelArgsBuilder::Build(
    const proto::GrpcOptions& options) {
    ::grpc::ChannelArguments args;
    const int recv = options.max_receive_message_bytes() > 0
                         ? options.max_receive_message_bytes()
                         : kDefaultMaxMessageBytes;
    const int send = options.max_send_message_bytes() > 0
                         ? options.max_send_message_bytes()
                         : kDefaultMaxMessageBytes;
    args.SetMaxReceiveMessageSize(recv);
    args.SetMaxSendMessageSize(send);
    if (options.keepalive_time_ms() > 0) {
        args.SetInt(GRPC_ARG_KEEPALIVE_TIME_MS, options.keepalive_time_ms());
    }
    if (options.keepalive_timeout_ms() > 0) {
        args.SetInt(GRPC_ARG_KEEPALIVE_TIMEOUT_MS,
                    options.keepalive_timeout_ms());
    }
    if (options.permit_keepalive_without_calls()) {
        args.SetInt(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
    }
    return args;
}

}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
