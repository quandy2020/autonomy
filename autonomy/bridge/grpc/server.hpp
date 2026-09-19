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
 * @file server.hpp
 * @brief Concrete gRPC bridge server implementing ServerInterface.
 *
 * @details
 * Owns the Autolink node, WorkScheduler, Context (installed as async_grpc
 * ExecutionContext), and the async_grpc::Server that hosts domain handlers.
 * Start() builds Context and registers handlers; Shutdown() joins threads
 * before Context / pool teardown.
 *
 * @see ServerInterface
 * @see Context
 * @see WorkScheduler
 */

#pragma once

#include <memory>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/server_interface.hpp"
#include "autonomy/bridge/grpc/work_scheduler.hpp"
#include "autonomy/bridge/proto/bridge_options.pb.h"
#include "autonomy/common/async_grpc/execution_context.h"
#include "autonomy/common/async_grpc/server.h"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Concrete gRPC bridge server implementing ServerInterface.
 *
 * Owns the Autolink node, WorkScheduler thread pool, bridge Context
 * (installed as async_grpc ExecutionContext), and the async_grpc::Server
 * that hosts all AutonomyService / domain handlers.
 *
 * @par Invariants
 * - Owns node, WorkScheduler, Context, and async_grpc::Server.
 * - Start registers handlers then runs the server; Shutdown tears down
 * threads before destroying Context / pool.
 * - Options / identity / capabilities are fixed at construction.
 * - configured_ prevents double Start configuration of the async_grpc server.
 *
 * @par Threading
 * Start / Shutdown / Wait* are control-plane calls. RPC handlers
 * run on async_grpc threads and must Schedule blocking Action waits onto
 * WorkScheduler rather than blocking the completion queue.
 *
 * @par Ownership
 * prefer Server::SharedPtr; BridgeServer / process main typically
 * holds the instance until process exit.
 */
class Server : public ServerInterface
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(Server)

    /**
     * @brief Construct a gRPC bridge server.
     *
     * Does not bind or start threads; call Start() after construction.
     * Creates the Autolink node and WorkScheduler from @p options (thread
     * counts / bind address live in GrpcOptions).
     *
     * @param[in] options      gRPC bind / TLS / handler options.
     * @param[in] identity     Fleet / inventory fields for GetRobotFullInfo.
     * @param[in] capabilities Optional capability advertise overrides.
     */
    explicit Server(const proto::GrpcOptions& options,
                    proto::RobotIdentityOptions identity = {},
                    proto::CapabilitiesOptions capabilities = {});

    /**
     * @brief Destructor; default — callers should Shutdown() before destroy
     * to join server threads cleanly.
     */
    ~Server() = default;

    /**
     * @brief Start the gRPC server.
     *
     * Builds Context, registers domain handlers, configures and starts
     * async_grpc::Server. Sets configured_ on success.
     *
     * @return false when bind or setup fails.
     *
     * @note Subsequent Start after a successful configuration is a no-op /
     * rejected depending on implementation; treat Start as once-per-life.
     */
    bool Start() final;

    /**
     * @brief Block until the server shuts down.
     *
     * Forwards to async_grpc::Server::WaitForShutdown. Another thread (or
     * signal handler) must call Shutdown() or the process will hang.
     */
    void WaitForShutdown() final;

    /**
     * @brief Wait until all computation is finished (for testing).
     *
     * Drains async_grpc in-flight work; use in unit / integration tests that
     * assert handler completion without tearing down the whole process.
     */
    void WaitUntilIdle() final;

    /**
     * @brief Shut down the gRPC server thread.
     *
     * Stops accepting new RPCs and joins async_grpc threads. Prefer calling
     * before destroying Server so Context / WorkScheduler outlive in-flight
     * Schedule callbacks.
     */
    void Shutdown() final;

private:
    /**
     * @brief gRPC bind / TLS / thread options fixed at construction.
     */
    const proto::GrpcOptions options_;

    /**
     * @brief Fleet / inventory identity passed into Context for System RPCs.
     */
    const proto::RobotIdentityOptions identity_;

    /**
     * @brief Capability advertise overrides passed into Context.
     */
    const proto::CapabilitiesOptions capabilities_;

    /**
     * @brief True after a successful Start() configured the async_grpc server.
     */
    bool configured_{false};

    /**
     * @brief Owned Autolink node shared with Context / domain stubs.
     */
    std::shared_ptr<autolink::Node> autolink_node_{nullptr};

    /**
     * @brief Owned background work pool for Action waits / Session Execute.
     */
    WorkScheduler::SharedPtr work_scheduler_{nullptr};

    /**
     * @brief Owned async_grpc::Server hosting registered RpcHandlers.
     */
    std::unique_ptr<autonomy::common::async_grpc::Server> grpc_server_{nullptr};
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
