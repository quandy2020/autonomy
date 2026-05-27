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
#include <chrono>
#include <thread>

#include "autonomy/common/gflags.hpp"
#include "autonomy/common/version.hpp"
#include "autonomy/system/options.hpp"
#include "autonomy/system/autonomy.hpp"
#include "autonomy/tasks/options.hpp"

namespace autonomy {
namespace system {
namespace {

std::atomic<bool> g_running{true};

void SigintHandler(int /*sig*/) {
    LOG(INFO) << "Shutdown autonomy system all tasks.";
    g_running = false;
}

void Run() {
    signal(SIGINT, SigintHandler);
    signal(SIGTERM, SigintHandler);

    autonomy::common::ShowVersion();
    LOG(INFO) << "Autonomy open robot for everyone enjoy !!!";

    const auto options = autonomy::system::CreateOptions(
        autonomy::common::FLAGS_configuration_directory,
        autonomy::common::FLAGS_configuration_basename);

    auto autonomy = autonomy::system::CreateAutonomy(options);
    autonomy->Start();

    tasks::RuntimeOptions runtime;
    runtime.config_directory = autonomy::common::FLAGS_configuration_directory;
    runtime.enable_bt_tasks = true;
    runtime.use_bt_navigation = true;
    if (options.has_task_options()) {
        const auto& task_opts = options.task_options();
        if (!task_opts.global_frame().empty()) {
            runtime.global_frame = task_opts.global_frame();
        }
        if (!task_opts.default_planner_id().empty()) {
            runtime.planner_id = task_opts.default_planner_id();
        }
        if (!task_opts.default_controller_id().empty()) {
            runtime.controller_id = task_opts.default_controller_id();
        }
        if (!task_opts.default_goal_checker_id().empty()) {
            runtime.goal_checker_id = task_opts.default_goal_checker_id();
        }
        if (task_opts.goal_reached_tolerance() > 0.0) {
            runtime.goal_tolerance = task_opts.goal_reached_tolerance();
        }
    }
    autonomy->Configure(runtime);

    LOG(INFO) << "Autonomy system running (BT ready="
              << (autonomy->IsReady() ? "yes" : "no") << "). "
              << "Press Ctrl+C to exit.";

    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

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
        return EXIT_SUCCESS;
    }

    if (autonomy::common::FLAGS_configuration_directory.empty() ||
        autonomy::common::FLAGS_configuration_basename.empty()) {
        AERROR << "Configuration directory or configuration basename is empty.";
        return EXIT_FAILURE;
    }

    autonomy::system::Run();
    google::ShutdownGoogleLogging();
    return EXIT_SUCCESS;
}
