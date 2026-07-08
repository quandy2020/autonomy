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

constexpr char kConfigDirEnv[] = "AUTONOMY_CONFIGURATION_DIRECTORY";
constexpr char kConfigBasenameEnv[] = "AUTONOMY_CONFIGURATION_BASENAME";
constexpr char kDefaultConfigBasename[] = "bridge/bridge_options.lua";

std::string ConfigurationDirectory() {
    if (!autonomy::common::FLAGS_configuration_directory.empty()) {
        return autonomy::common::FLAGS_configuration_directory;
    }
    const char* env = std::getenv(kConfigDirEnv);
    return env != nullptr ? std::string(env) : std::string();
}

std::string ConfigurationBasename() {
    if (!autonomy::common::FLAGS_configuration_basename.empty()) {
        return autonomy::common::FLAGS_configuration_basename;
    }
    const char* env = std::getenv(kConfigBasenameEnv);
    return env != nullptr ? std::string(env) : std::string(kDefaultConfigBasename);
}

void SigintHandler(int /*sig*/) {
    LOG(INFO) << "Shutdown autonomy bridge.";
    autolink::AsyncShutdown();
}

int Run() {
    if (ConfigurationDirectory().empty()) {
        LOG(ERROR) << "configuration_directory is required (--configuration_directory "
                      "or " << kConfigDirEnv << ").";
        return EXIT_FAILURE;
    }

    if (ConfigurationBasename().empty()) {
        LOG(ERROR) << "configuration_basename is required (--configuration_basename "
                      "or " << kConfigBasenameEnv << ").";
        return EXIT_FAILURE;
    }

    autonomy::common::ShowVersion();
    LOG(INFO) << "Starting autonomy bridge (gRPC / MQTT external API).";

    const auto options =
        common::CreateOptions(ConfigurationDirectory(), ConfigurationBasename());

    BridgeServer server(options);
    if (!server.Start()) {
        LOG(ERROR) << "Failed to start bridge server.";
        return EXIT_FAILURE;
    }
    LOG(INFO) << "Bridge server running. Press Ctrl+C to exit.";
    autolink::WaitForShutdown();
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
        "  autonomy_bridge \\\n"
        "    --configuration_directory=config \\\n"
        "    --configuration_basename=bridge/bridge_options.lua\n");

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
