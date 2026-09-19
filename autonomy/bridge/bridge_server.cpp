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

/**
 * @file bridge_server.cpp
 * @brief Implementation of BridgeServer lifecycle (Start / Wait / Shutdown).
 */

#include "autonomy/bridge/bridge_server.hpp"

#include <autonomy/common/port.hpp>

#include "autolink/common/log.hpp"
#include "autonomy/common/json_util.hpp"

namespace autonomy {
namespace bridge {

BridgeServer::BridgeServer() {
    grpc_bridge_ = std::make_unique<grpc::Server>(proto::GrpcOptions{});
}

BridgeServer::BridgeServer(const proto::BridgeOptions& options)
    : options_{options} {
    grpc_bridge_ = std::make_unique<grpc::Server>(
        options_.grpc(), options_.identity(), options_.capabilities());
}

bool BridgeServer::Start() {
    if (!grpc_bridge_) {
        AERROR << "BridgeServer: gRPC bridge missing.";
        return false;
    }

    AINFO << "Use gRPC as bridge communication.";
    return grpc_bridge_->Start();
}

void BridgeServer::WaitForShutdown() {
    if (grpc_bridge_) {
        grpc_bridge_->WaitForShutdown();
    }
}

void BridgeServer::Shutdown() {
    if (grpc_bridge_) {
        grpc_bridge_->Shutdown();
    }
}

}  // namespace bridge
}  // namespace autonomy
