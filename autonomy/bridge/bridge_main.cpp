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
 * @file bridge_main.cpp
 * @brief Process entry: CLI11 → serve | call | list | describe.
 */

#include <signal.h>

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <string>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/bridge/bridge_server.hpp"
#include "autonomy/bridge/options.hpp"

namespace autonomy {
namespace bridge {
namespace {

std::atomic<bool> g_shutdown_requested{false};

void SigintHandler(int /*sig*/) {
    bool expected = false;
    if (!g_shutdown_requested.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
        return;
    }
    autolink::AsyncShutdown();
}

int RunServe(const CliOptions& cli) {
    auto options = CreateOptions(cli.conf_file);
    ApplyCliOverrides(cli, &options);

    if (cli.print_config) {
        std::cout << options.DebugString() << std::flush;
    }

    AINFO << "Bridge options: " << SummarizeOptions(options);

    if (cli.dry_run) {
        AINFO << "Dry-run: conf loaded OK; not starting server.";
        std::cout << SummarizeOptions(options) << '\n';
        return EXIT_SUCCESS;
    }

    if (cli.self_test) {
        std::string detail;
        if (!RunPlatformSelfTest(options, &detail)) {
            AERROR << "Self-test failed: " << detail;
            return EXIT_FAILURE;
        }
        AINFO << "Self-test OK: " << detail;
        std::cout << "self-test: OK\n" << detail << '\n';
        return EXIT_SUCCESS;
    }

    BridgeServer server(options);
    if (!server.Start()) {
        AERROR << "Failed to start bridge server.";
        return EXIT_FAILURE;
    }
    AINFO << "Bridge server running. Press Ctrl+C to exit.";
    autolink::WaitForShutdown();
    if (g_shutdown_requested.load(std::memory_order_acquire)) {
        AINFO << "Shutdown autonomy bridge.";
    }
    server.Shutdown();
    return EXIT_SUCCESS;
}

}  // namespace
}  // namespace bridge
}  // namespace autonomy

int main(int argc, char** argv) {
    autonomy::bridge::CliOptions cli;
    const auto status =
        autonomy::bridge::ParseCommandLine(argc, argv, &cli);
    if (status == autonomy::bridge::ParseStatus::kExitOk) {
        return EXIT_SUCCESS;
    }
    if (status == autonomy::bridge::ParseStatus::kExitError) {
        return EXIT_FAILURE;
    }

    // Client modes: no Autolink node required.
    if (cli.mode != autonomy::bridge::CliMode::kServe) {
        return autonomy::bridge::RunRpcClientMode(cli);
    }

    if (!autolink::Init(argv[0])) {
        AERROR << "autolink::Init failed.";
        return EXIT_FAILURE;
    }

    signal(SIGINT, autonomy::bridge::SigintHandler);
    signal(SIGTERM, autonomy::bridge::SigintHandler);

    const int exit_code = autonomy::bridge::RunServe(cli);
    autolink::Clear();
    return exit_code;
}
