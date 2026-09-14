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

#include <glog/logging.h>
#include <signal.h>

#include <atomic>
#include <cstdlib>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/bridge/bridge_server.hpp"
#include "autonomy/bridge/common/bridge_interface.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/version.hpp"

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

int Run() {
    autonomy::common::ShowVersion();
    LOG(INFO) << "Starting autonomy bridge (gRPC / MQTT external API).";

    const std::string conf = autonomy::common::FLAGS_conf.empty()
                                 ? std::string("bridge.pb.txt")
                                 : autonomy::common::FLAGS_conf;
    const auto options = common::CreateOptions(conf);

    BridgeServer server(options);
    if (!server.Start()) {
        LOG(ERROR) << "Failed to start bridge server.";
        return EXIT_FAILURE;
    }
    LOG(INFO) << "Bridge server running. Press Ctrl+C to exit.";
    autolink::WaitForShutdown();
    if (g_shutdown_requested.load(std::memory_order_acquire)) {
        LOG(INFO) << "Shutdown autonomy bridge.";
    }
    server.Shutdown();
    return EXIT_SUCCESS;
}

}  // namespace
}  // namespace bridge
}  // namespace autonomy

int main(int argc, char** argv) {
    google::SetUsageMessage(
        "\n\n"
        "\033[31m External bridge process (gRPC AutonomyService).\033[0m \n"
        "Example:\n"
        "  autonomy.bridge --conf=bridge.pb.txt\n"
        "  # or AUTONOMY_PATH=/path/to/prefix\n");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (autonomy::common::FLAGS_verbose) {
        autonomy::common::ShowVersion();
        return EXIT_SUCCESS;
    }

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed.";
        google::ShutdownGoogleLogging();
        return EXIT_FAILURE;
    }

    signal(SIGINT, autonomy::bridge::SigintHandler);
    signal(SIGTERM, autonomy::bridge::SigintHandler);

    const int exit_code = autonomy::bridge::Run();
    autolink::Clear();
    google::ShutdownGoogleLogging();
    return exit_code;
}
