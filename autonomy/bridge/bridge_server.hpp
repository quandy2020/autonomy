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
 * @file bridge_server.hpp
 * @brief Top-level bridge process host wrapping the gRPC Server.
 *
 * @details
 * Process / Component entry owns BridgeServer. It constructs a
 * grpc::Server from BridgeOptions (grpc / identity / capabilities) and
 * exposes Start / WaitForShutdown / Shutdown for main and signal paths.
 *
 * @see grpc::Server
 * @see CreateOptions
 */

#pragma once

#include <memory>
#include <unordered_map>

#include "autonomy/bridge/grpc/server.hpp"
#include "autonomy/bridge/proto/bridge_options.pb.h"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {

/**
 * @brief Top-level bridge process host for the gRPC transport.
 *
 * Owns a gRPC Server configured from BridgeOptions. Prefer constructing
 * with parsed options from CreateOptions(); the default ctor uses empty
 * GrpcOptions (tests / minimal boot).
 *
 * @par Ownership
 * Sole owner of grpc_bridge_ UniquePtr; destroy after Shutdown().
 *
 * @par Threading
 * Start / WaitForShutdown / Shutdown are control-plane calls from main
 * or Component Clear; do not invoke from gRPC handler threads.
 */
class BridgeServer
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(BridgeServer)

    /**
     * @brief Construct with default (empty) BridgeOptions.
     *
     * Builds a gRPC Server from default-constructed GrpcOptions /
     * identity / capabilities nested fields.
     */
    explicit BridgeServer();

    /**
     * @brief Construct from parsed BridgeOptions.
     *
     * @param[in] options Bridge-wide options (grpc bind, identity,
     *                    capabilities).
     */
    explicit BridgeServer(const proto::BridgeOptions& options);

    /**
     * @brief Destructor; callers should Shutdown() before destroy.
     */
    ~BridgeServer() = default;

    /**
     * @brief Start the owned gRPC bridge Server.
     *
     * @return true on success; false on setup / bind failure.
     */
    bool Start();

    /**
     * @brief Block until the gRPC server stops.
     *
     * @warning Blocking; another thread must call Shutdown() or the
     * process hangs.
     */
    void WaitForShutdown();

    /**
     * @brief Stop gRPC / background threads (Component Clear / signal path).
     */
    void Shutdown();

private:
    /**
     * @brief Bridge options copied at construction (grpc / identity / caps).
     */
    const proto::BridgeOptions options_;

    /**
     * @brief Owned concrete gRPC Server (null until constructed in ctors).
     */
    grpc::Server::UniquePtr grpc_bridge_{nullptr};
};

}  // namespace bridge
}  // namespace autonomy
