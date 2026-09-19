/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_probe.hpp
 * @brief Generic RPC client for CLI: list / describe / call any automsgs.rpcs method.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace tools {

/**
 * @brief Options for a single @ref CallRpc invocation.
 */
struct RpcCallOptions {
    AUTONOMY_SMART_PTR_DEFINITIONS(RpcCallOptions)

    /** @brief `host:port` (default 127.0.0.1:5005). */
    std::string target{"127.0.0.1:5005"};

    /** @brief Request body as protobuf JSON (`{}` if empty). */
    std::string json_data{"{}"};

    /** @brief Raw metadata lines `key:value`. */
    std::vector<std::string> headers;

    /** @brief Optional Bearer token → `authorization: Bearer …`. */
    std::string bearer_token;

    /** @brief Optional `x-robot-id` metadata. */
    std::string robot_id;

    /** @brief Per-RPC deadline in seconds (0 = no deadline). */
    int timeout_sec{30};

    /** @brief Use TLS (insecure by default). */
    bool tls{false};

    /** @brief Print wire method path to stderr. */
    bool verbose{false};
};

/**
 * @brief Result of @ref CallRpc / catalog helpers.
 */
struct RpcProbeResult {
    AUTONOMY_SMART_PTR_DEFINITIONS(RpcProbeResult)

    bool ok{false};
    int exit_code{1};
    std::string message;
};

/**
 * @brief Ensure linked automsgs.rpcs descriptors are registered (call once).
 */
void EnsureRpcDescriptorsLinked();

/**
 * @brief Resolve `Service/Method`, short Method, or full path → `/pkg.Svc/Method`.
 */
bool ResolveRpcMethod(const std::string& raw, std::string* full_path,
                      std::string* error);

/**
 * @brief Print services/methods to stdout (optional service filter).
 */
RpcProbeResult ListRpcMethods(const std::string& service_filter);

/**
 * @brief Print descriptor detail for a service, method, or message type.
 */
RpcProbeResult DescribeRpcSymbol(const std::string& symbol);

/**
 * @brief Call any unary or server-streaming RPC with JSON request body.
 *
 * Client-streaming / bidi are rejected with a clear error (use grpcurl).
 */
RpcProbeResult CallRpc(const std::string& method, const RpcCallOptions& options);

}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
