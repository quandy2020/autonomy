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
 * @file server_interface.hpp
 * @brief Abstract lifecycle interface for the bridge gRPC server.
 *
 * @details
 * Narrow Start / WaitForShutdown / WaitUntilIdle / Shutdown API for tests and
 * BridgeServer wrappers. Concrete Server owns async_grpc runtime, Context, and
 * WorkScheduler.
 *
 * @see Server
 * @see BridgeServer
 */

#pragma once

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Abstract lifecycle interface for the bridge gRPC server.
 *
 * Separates start / wait / idle / shutdown from the concrete Server so tests
 * and BridgeServer wrappers can depend on a narrow API. Implementations own
 * the async_grpc runtime, Context, and WorkScheduler.
 *
 * @par Threading
 * Start / Shutdown are expected to be called from a single control
 * thread; WaitForShutdown blocks until Shutdown (or peer close) completes.
 * WaitUntilIdle is for tests that need to drain in-flight RPCs / pool work.
 *
 * @par Ownership
 * polymorphic; prefer SharedPtr aliases from
 * AUTONOMY_SMART_PTR_DEFINITIONS. make_shared only instantiates if called on
 * a concrete derived type.
 */
class ServerInterface
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ServerInterface)

    /**
     * @brief Virtual destructor for ServerInterface.
     */
    virtual ~ServerInterface() {}

    /**
     * @brief Start the gRPC server.
     *
     * Registers handlers, binds the listening address, and starts the
     * async_grpc completion / worker threads as required by the concrete type.
     *
     * @return false when bind or setup fails (server remains unusable).
     *
     * @note Idempotent Start behavior is implementation-defined; Server tracks
     * a configured_ flag and refuses double configuration.
     */
    virtual bool Start() = 0;

    /**
     * @brief Wait for the server to shut down.
     *
     * The server must be either shutting down or some other thread must call
     * Shutdown() for this function to ever return.
     *
     * @warning Blocking call; do not invoke from a gRPC handler thread.
     */
    virtual void WaitForShutdown() = 0;

    /**
     * @brief Wait until all computation is finished (for testing).
     *
     * Drains in-flight handler work / pool tasks as defined by the concrete
     * Server. Production code normally uses WaitForShutdown instead.
     */
    virtual void WaitUntilIdle() = 0;

    /**
     * @brief Shut down the gRPC server thread.
     *
     * Signals the async_grpc runtime to stop accepting work and join threads.
     * Safe to call concurrent with WaitForShutdown (unblocks it).
     *
     * @note After Shutdown, Start must not be assumed re-entrant unless the
     * concrete type documents otherwise.
     */
    virtual void Shutdown() = 0;
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
