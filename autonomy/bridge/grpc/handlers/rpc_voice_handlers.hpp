/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/common/async_grpc/rpc_handler.h"
#include <automsgs/rpcs/voice.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    RpcVoiceExecuteSignature, ::automsgs::rpcs::voice::VoiceCommandRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::voice::VoiceCommandResponse>,
    "/automsgs.rpcs.voice.VoiceService/Execute")

class RpcVoiceExecuteHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcVoiceExecuteSignature>
{
public:
    void OnRequest(
        const ::automsgs::rpcs::voice::VoiceCommandRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

