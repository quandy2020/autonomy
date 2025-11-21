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

#include "autonomy/common/async_grpc/execution_context.h"
#include "autonomy/common/async_grpc/server.h"
#include "autonomy/bridge/proto/bridge_options.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/bridge/common/bridge_interface.hpp"
#include "autonomy/bridge/plugins//grpc/grpc_bridge_server_interface.hpp"

namespace autonomy {
namespace bridge { 
namespace plugins {
namespace grpc {

class GrpcBridgeServer : public GrpcBridgeServerInterface
{
public:

    /**
     * Define GrpcBridgeServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(GrpcBridgeServer)

    GrpcBridgeServer();
    ~GrpcBridgeServer() = default;

    // Starts the gRPC server
    void Start() final;

    // Waits for the 'GrpcBridgeServer' to shut down. Note: The server must be
    // either shutting down or some other thread must call 'Shutdown()' for this
    // function to ever return.
    void WaitForShutdown() final;

    // Waits until all computation is finished (for testing).
    void WaitUntilIdle() final;

    // Shuts down the gRPC server, the 'LocalTrajectoryUploader' and the SLAM
    // thread.
    void Shutdown() final;

private:

    void ProcessSensorDataQueue();
    void StartThread();

    ///> grpc options
    const proto::GrpcOptions options_;

    bool shutting_down_{false};
    std::unique_ptr<std::thread> task_thread_;
    std::unique_ptr<autonomy::common::async_grpc::Server> grpc_server_{nullptr};
};

}   // namespace grpc
}   // namespace plugins 
}   // namespace bridge
}   // namespace autonomy