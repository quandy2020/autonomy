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

#include "autonomy/common/gflags.hpp"
#include "autonomy/common/version.hpp"
#include "autonomy/system/options.hpp"
#include "autonomy/system/system.hpp"

namespace autonomy {
namespace system {
namespace {

void SigintHandler(int sig) {
    LOG(INFO) << "Shutdown autonomy system all tasks.";
    exit(0);
}

void Run() {
    // 'Crtl + C' sign handler
    signal(SIGINT, SigintHandler);
    signal(SIGTERM, SigintHandler);

    // Show autonomu app version
    autonomy::common::ShowVersion();
    LOG(INFO) << "Autonomy open robot for everyone enjoy !!!";

    // Create options from lua
    auto options = autonomy::system::CreateOptions(
        autonomy::common::FLAGS_configuration_directory,
        autonomy::common::FLAGS_configuration_basename);

    // // Create autonomy node
    // auto autonomy = autonomy::system::CreateAutonomy(options);
    // autonomy->Start();

    // Shutdown autonomy node
    autonomy->Shutdown();
}

}  // namespace
}  // namespace system
}  // namespace autonomy

int main(int argc, char** argv) {
    google::SetUsageMessage(
        "\n\n"
        "\033[31m This program offers autonomy framework development for "
        "robot.\033[0m \n");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (autonomy::common::FLAGS_verbose) {
        autonomy::common::ShowVersion();
        exit(0);
    }

    // Check if configuration directory and basename are set.
    if (autonomy::common::FLAGS_configuration_directory.empty() ||
        autonomy::common::FLAGS_configuration_basename.empty()) {
        AERROR << "Configuration directory or configuration basename is empty.";
        return EXIT_FAILURE;
    }

    autonomy::system::Run();
    google::ShutdownGoogleLogging();
    return EXIT_SUCCESS;
}