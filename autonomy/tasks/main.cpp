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
#include <csignal>
#include <cstdlib>

#include "autolink/autolink.hpp"
#include "autolink/init.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/version.hpp"
#include "autonomy/tasks/navigator/bt_navigator.hpp"
#include "autonomy/tasks/options.hpp"

namespace autonomy {
namespace tasks {
namespace {

void SigintHandler(int sig) {
    LOG(INFO) << "Shutdown autonomy tasks (behavior-tree navigators).";
    exit(0);
}

void Run() {
    // 'Ctrl + C' signal handler
    signal(SIGINT, SigintHandler);
    signal(SIGTERM, SigintHandler);

    // Show autonomy app version
    autonomy::common::ShowVersion();
    LOG(INFO) << "Autonomy tasks entry (behavior-tree based navigation).";

    // Create options from lua
    auto options = autonomy::tasks::CreateOptions(autonomy::common::FLAGS_configuration_directory,
                                                  autonomy::common::FLAGS_configuration_basename);
    // Create node
    auto node = ::autolink::CreateNode("navigator_node");

    // Create navigator
    auto navigator = autonomy::tasks::navigator::BtNavigator(node, options);
}

}  // namespace
}  // namespace tasks
}  // namespace autonomy

int main(int argc, char** argv) {
    ::autolink::Init(argv[0]);
    google::SetUsageMessage(
        "\n\n"
        "\033[31m This program runs autonomy tasks (behavior-tree navigators) for "
        "robot.\033[0m \n");

    // autolink::Init() already calls google::InitGoogleLogging(); do not call it again.
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (autonomy::common::FLAGS_verbose) {
        autonomy::common::ShowVersion();
        exit(0);
    }

    // Check if configuration directory and basename are set.
    if (autonomy::common::FLAGS_configuration_directory.empty() ||
        autonomy::common::FLAGS_configuration_basename.empty()) {
        AERROR << "Configuration directory or configuration basename is empty.";
        AERROR << "Usage: " << argv[0] << " --configuration_directory <dir> --configuration_basename <file.lua>";
        AERROR << "Example: " << argv[0] << " --configuration_directory /path/to/configuration_files"
               << " --configuration_basename task_options.lua";
        return EXIT_FAILURE;
    }

    autonomy::tasks::Run();
    google::ShutdownGoogleLogging();
    ::autolink::Clear();
    return EXIT_SUCCESS;
}
