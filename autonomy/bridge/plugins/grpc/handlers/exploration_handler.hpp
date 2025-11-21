/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "google/protobuf/empty.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/common/async_grpc/rpc_handler.h"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/bridge/proto/external_command_service.grpc.pb.h"


namespace autonomy {
namespace bridge { 
namespace plugins { 
namespace grpc { 
namespace handlers { 

DEFINE_HANDLER_SIGNATURE(
    SendExplorationSignature, 
    proto::ExplorationCommandRequest,
    autonomy::common::async_grpc::Stream<proto::ExplorationCommandResponse>,
    "/autonomy.bridge.proto.AutonomyService/SendExplorationCommand")

class SendExplorationHandler
    : public autonomy::common::async_grpc::RpcHandler<SendExplorationSignature> {
public:
    void OnRequest(const proto::ExplorationCommandRequest& request) override;

    void OnReadsDone() override {
        // this->Send(std::make_unique<google::protobuf::Empty>());
    }
};

}   // namespace handlers
}   // namespace grpc
}   // namespace plugins
}   // namespace bridge
}   // namespace autonomy