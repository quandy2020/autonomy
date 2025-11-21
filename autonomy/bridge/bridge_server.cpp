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

#include "autonomy/bridge/bridge_server.hpp"

#include <autonomy/common/port.hpp>
#include "autonomy//common/logging.hpp"
#include "autonomy/common/json_util.hpp"

namespace autonomy {
namespace bridge {

BridgeServer::BridgeServer()
{
    grpc_bridge_ = std::make_unique<plugins::grpc::GrpcBridgeServer>();
}

BridgeServer::BridgeServer(const proto::BridgeOptions& options)
    : options_{options}
{
}


void BridgeServer::Start()
{
    if (options_.use_grpc()) {
        LOG(INFO) << "Use mqtt gRPC as communication.";
        // grpc_bridge_->Start();
    }

    if (options_.use_mqtt()) {
        LOG(INFO) << "Use mqtt bridge as communication.";
    }

    grpc_bridge_->Start();
}

void BridgeServer::WaitForShutdown()
{
    grpc_bridge_->WaitForShutdown();
}
    

}   // namespace bridge
}   // namespace autonomy

