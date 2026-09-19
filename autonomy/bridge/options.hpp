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
 * @file options.hpp
 * @brief Bridge process CLI (CLI11): serve / call / list / describe + conf.
 *
 * @see BridgeServer
 * @see CreateOptions
 * @see ParseCommandLine
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/bridge/proto/bridge_options.pb.h"

namespace autonomy {
namespace bridge {

/**
 * @brief Top-level CLI mode.
 */
enum class CliMode {
    kServe = 0,     ///< Start gRPC server (default)
    kCall = 1,      ///< Call an RPC with JSON params
    kList = 2,      ///< List services / methods
    kDescribe = 3,  ///< Describe service / method / message
};

/**
 * @brief Process-level CLI options for `autonomy.bridge`.
 */
struct CliOptions {
    CliMode mode{CliMode::kServe};

    /** @brief Conf basename or path (default: bridge.pb.txt). */
    std::string conf_file{"bridge.pb.txt"};

    /**
     * @brief Load conf, print summary, exit without listening
     *        (`-n` / `--dry-run`).
     */
    bool dry_run{false};

    /**
     * @brief Load conf + run ApplyPlatform self-check, then exit
     *        (`-t` / `--self-test`). Serve-mode only.
     */
    bool self_test{false};

    /** @brief After load, print BridgeOptions DebugString. */
    bool print_config{false};

    /** @brief When set, override `grpc.host` (serve) or call target host. */
    bool host_set{false};
    std::string host;

    /** @brief When set, override `grpc.port`. */
    bool port_set{false};
    std::uint32_t port{0};

    // --- call / list / describe ---

    /** @brief Method (`SystemService/Heartbeat`) or list filter / describe symbol. */
    std::string rpc_symbol;

    /** @brief JSON body or `@file` for `call -d`. */
    std::string rpc_data{"{}"};

    /** @brief Explicit `host:port` for client modes (overrides host/port). */
    std::string rpc_target;

    /** @brief Metadata `key:value` (repeatable). */
    std::vector<std::string> rpc_headers;

    /** @brief Bearer token for call. */
    std::string rpc_bearer;

    /** @brief `x-robot-id` for call. */
    std::string rpc_robot_id;

    /** @brief Call deadline seconds. */
    int rpc_timeout_sec{30};

    /** @brief Use TLS for call. */
    bool rpc_tls{false};

    /** @brief Verbose client logging. */
    bool rpc_verbose{false};
};

/**
 * @brief Result of @ref ParseCommandLine.
 */
enum class ParseStatus {
    kRun = 0,       ///< Continue into Run()
    kExitOk = 1,    ///< Help / version printed; exit 0
    kExitError = 2  ///< Parse failure; exit non-zero
};

/**
 * @brief Colored / plain version banner for `-V` / `--version`.
 */
std::string VersionString();

/**
 * @brief Parse argv with CLI11 (serve options + call/list/describe).
 */
ParseStatus ParseCommandLine(int argc, char** argv, CliOptions* out);

/**
 * @brief Apply host/port CLI overrides onto loaded BridgeOptions.
 */
void ApplyCliOverrides(const CliOptions& cli, proto::BridgeOptions* options);

/**
 * @brief Resolve client target `host:port` from CLI / conf defaults.
 */
std::string ResolveRpcTarget(const CliOptions& cli);

/**
 * @brief Load bridge configuration from a protobuf text file.
 */
proto::BridgeOptions CreateOptions(
    const std::string& conf_file = "bridge.pb.txt");

/**
 * @brief One-line summary of grpc bind + platform switches.
 */
std::string SummarizeOptions(const proto::BridgeOptions& options);

/**
 * @brief Run ApplyPlatform against a throwaway Builder.
 */
bool RunPlatformSelfTest(const proto::BridgeOptions& options,
                         std::string* detail);

/**
 * @brief Execute call / list / describe modes; return process exit code.
 */
int RunRpcClientMode(const CliOptions& cli);

}  // namespace bridge
}  // namespace autonomy
